import logging
import logging.config
from threading import Thread
from threading import Event
from queue import Queue
from queue import Empty
import RPi.GPIO as GPIO
import sys
import time
import subprocess
import requests as _requests
import re
import os
from datetime import datetime
from pathlib import Path

from aq_lib.meerstetter import MeerStetter
from aq_lib.meerstetter import set_time, get_time
from aq_lib.thermal_engine import RunStopped, thermal_engine
from aq_lib.thermal_parser import thermal_parser
from aq_lib.config_module import Config
from aq_lib.utils import load_json
from aq_lib.utils import LogFileName
from aq_lib.utils import LOGGING_CONFIG
from aq_curve.plot_utils import generate_optics_plot
from aq_lib.regulate import lid_heater_worker
from aq_lib import lid_worker_metrics as lwm
from config import get_src_basedir
import aq_lib.state_requests as sr
from aq_lib.motor_class import Axis, Drawer

from aq_curve.main import results_to_json
from aq_lib.fan_class import Fan
from aq_lib.adc_class import OpticalRead
from aq_lib.thermal_parser import count_optics_passes
from aq_lib.optics_read_plan import READS_PER_CYCLE, optics_read_tasks
from aquila_web.optics_readings import expected_lines

logging.config.dictConfig( LOGGING_CONFIG )
# Attach the dedicated homing log (its own rotated JSON-lines file, kept out of
# logger.log). Motors home from this process, so wire it up here (ADR-021, #325).
from aq_lib.homing_log import configure_homing_logger
configure_homing_logger()
logger = logging.getLogger( "aquila" )
config = Config()

class AssayInterface():

    def __init__( self ):

        self.updated4 = False # this flag indicates whether legacy or new optics should be used
        value = config.info.get("updated4")
        if (
            value is not None
            and value != 0
            and not (
                isinstance(value, str)
                and value.strip().lower() in ("0", "false")
            )
        ):
            self.updated4 = True

        self.well_15 = False # this flag whether 4well or 15well arrangement should be used
        value = config.info.get("well_15")
        if (
            value is not None
            and value != 0
            and not (
                isinstance(value, str)
                and value.strip().lower() in ("0", "false")
            )
        ):
            self.well_15 = True

        self.axis = Axis()
        self.drawer = Drawer()
        self.pcr_fan = Fan()
        self.pcr_fan.set_state(1)

        self.message_queue = Queue()
        self.lid_heater_stop_event = Event()
        self.lid_heater_quiet_event = Event()


        self.optics = OpticalRead()
        self.optics.read_config()

        self.run_aborted = False
        self._run_index = 0

        self.configure_thermal_control()
         
        self.axis.home()
        self.axis.reset_position()
        self.drawer.home()
        sr.change_screen("0")
        sr.update_drawer_state(is_open=False, is_closed=True)

    def configure_thermal_control(self):
        device_type = int (config.pcr["device_type"] )
        pid = int(config.pcr["pid"], 16)
        vid = int(config.pcr["vid"], 16)
        device = MeerStetter.find_meer( vid, pid, device_type)
        logger.debug ( "Temperature controller port: %s", device )
        self.meer = MeerStetter( device, baudrate = 57600, timeout = 1 )

        self.meer.setKp(80)
        self.meer.setTi(5)
        self.meer.setTd(4)

        self.thermal_profile = ""

    def executor( self ):
        logger.info( "Execution thread started." )
        while True:
            try:
                item = self.message_queue.get( timeout=10 ) 
                self.lid_heater_quiet_event.set()
                logger.info( "Task received: %s", item.__str__() )
                
                if type(item) is dict and "capture" in item:
                    logger.info("Capture task")
                    dye = item["capture"]
                    self.optics.set_channel_dye( dye )
                    self.optics.capture_blink( 
                                       dye, 
                                       item["cycle"],
                                       item["position"],
                                       )
                elif type(item) is dict and "move" in item:
                    self.axis.move_abs_wo_home_flag( item["move"], 0 )

                elif type(item) is dict and "home" in item:
                    ret = self.axis.home()
                    logger.info("Axis Position is %d", self.axis.position )
                    self.axis.reset_position()

                elif type(item) is dict and "goto_position" in item:
                    position = item["goto_position"]
                    ret = self.axis.goto_position( position )
                    self.drawer.goto_position( position )

                elif type(item) is str and item == "quit":
                    lfn = LogFileName()
                    run_prefix = self._safe_name(self.run_name)
                    self.optics.out_data() # does nothing if two_adc mode is disabled; otherwise - actually writes data out to the optics output files
                    break

                self.message_queue.task_done()  # Mark the task as complete
                continue

            except Empty:
                logger.debug( "Executor idle" )
                self.lid_heater_quiet_event.clear()
                self.optics.data_file.flush()

    def queue_task( self, item ):
        self.message_queue.put ( item )

    def read_wells( self, args ):
        cycle = args[1]  # name, n, last_temp...
        # One optical read pass per read_wells call; the capture pattern (and
        # thus READS_PER_CYCLE) lives in aq_lib.optics_read_plan so completeness
        # math can't drift from the blinks actually fired (#288).
        self._optics_pass_count += 1

        for task in optics_read_tasks(cycle, self.well_15, self.updated4):
            self.queue_task( task )

    def callback( self, args ):
        logger.info( "Callback" )
        if "pcr_fanoff" in args: 
            pass
            self.pcr_fan.set_state ( 0 )
        elif "pcr_fanon" in args: self.pcr_fan.set_state ( 1 )
        elif "optics" in args: 
            logger.info( "Optics" )
            self.read_wells( args )


    def ready( self ):
        #Ready Screen
        sr.change_screen("1")
        ret = self.button_logic( state = "ready" )
        if ret == None:
            ret = self.button_logic( state = "ready" )
            if ret == None:
                logger.error("Profile is returning none when it should not. %s" % (ret))
                sr.change_screen("-1")
                raise Exception ("Profile is returning none when it should not. %s" % (ret))
        profile, run_name = ret
        logger.info("Profile selected: %s" % (profile))
        self.run_name = run_name
        self.thermal_profile = ("profiles/" + profile)
        
    
    def run( self ):
        #Run screen
        self._run_index += 1
        # One canonical run_timestamp per Run (#287), captured once at run start.
        # Shared by run_complete and the forthcoming optics_readings event so the
        # cloud derives the same run_id = uuid5(device_id : run_timestamp).
        self.run_timestamp = datetime.utcnow().isoformat(timespec="seconds") + "Z"
        # Optical read passes for the optics_readings completeness check (#288).
        # _planned_optics_passes is the profile's intended total (set once steps
        # are loaded, below); _optics_pass_count is the runtime fallback.
        self._optics_pass_count = 0
        self._planned_optics_passes = 0
        logger.info("RUN START index=%d run_timestamp=%s", self._run_index, self.run_timestamp)
        sr.change_screen("2")
        time.sleep(1)
        sr.timer_control( status = "start" )
        self.run_aborted = False
        sr.reset_stop_request()

        # Drain any stale tasks/quit left over from a previous run so the
        # new executor starts with a clean queue.
        while not self.message_queue.empty():
            try:
                self.message_queue.get_nowait()
            except Empty:
                break

        stop_event = Event()
        stop_monitor_event = Event()
        stop_thread = Thread(
            target=self._monitor_stop_request,
            args=(stop_event, stop_monitor_event)
        )
        stop_thread.daemon = True
        stop_thread.start()

        self.drawer.read()
        lfn = LogFileName()
        run_prefix = self._safe_name(self.run_name)

        pcr_log = lfn.get_pcr_log_filename(prefix=f"{run_prefix}_")
        optics_log = lfn.get_optics_log_filename(prefix=f"{run_prefix}_")
        results_json = lfn.get_results_json_filename(prefix=f"{run_prefix}_")
        plot_filename = f"{run_prefix}_{lfn.id}.png"
        plot_path = os.path.join("logs/plots", plot_filename)

        logger.info( "PCR log: %s", pcr_log )
        logger.info( "Optics log: %s", optics_log )
        logger.info("Optics log absolute: %s", Path(optics_log).resolve())

        steps = load_json( self.thermal_profile )["steps"]
        # Planned optical passes for the whole run, from the profile — so optics
        # expected_lines reflects the intended total even on an early abort (#288).
        self._planned_optics_passes = count_optics_passes(steps)
        execution_thread = Thread( target = self.executor )

        execution_thread.daemon = True
        execution_thread.start()

        self.lid_thread = Thread ( 
             target = lid_heater_worker, 
             args = ( self.lid_heater_stop_event, self.lid_heater_quiet_event, ) 
        )
        self.lid_heater_stop_event.clear()
        self.lid_thread.start()
        self.lid_heater_quiet_event.clear()

        try:
            with (
                    open ( pcr_log,"w" ) as pcr_fp,
                    open ( optics_log, "w" ) as optics_fp
                    ):
                self.optics.data_file = optics_fp
                print ( "# Starting optics log", file = optics_fp, flush=True )
                t0_sync = set_time()
                print ( "# Starting log t0 = %f"% t0_sync, file = pcr_fp )
                actions = thermal_parser( steps )
                try:
                    thermal_engine( actions, self.meer, self.callback, pcr_fp, stop_event )
                finally:
                    # Drain the executor while files are still open so capture
                    # threads cannot write to a closed file handle.
                    # Use a generous timeout: a 40-cycle run with optics can
                    # take several seconds per capture position.
                    self.message_queue.put("quit")
                    execution_thread.join(timeout=60)

        except RunStopped:
            logger.info("Run stopped by user")
            self.run_aborted = True
        except KeyboardInterrupt as ki:
            logger.error ( "Keyboard Interrupt. Turning off Meerstetter controller. " )
            sr.change_screen("-3")
        except Exception as e:
            logger.error ( "Exception during thermocycling. Turning off Meerstetter controller. " )
            sr.change_screen("-1")
            raise ( e )
        finally:
            if execution_thread.is_alive():
                # Safety net: executor did not finish within the inner timeout
                self.message_queue.put("quit")
                execution_thread.join(timeout=2)
            self.hw_deinitialize()
            sr.timer_control("stop")
            if self.run_aborted:
                sr.timer_control("reset")
            self.drawer.open()
            # Stop monitor only after drawer is open — keeps the stop button
            # functional during teardown motor movement (which can take
            # 30-90 s if the drawer missed steps and needs to re-home).
            stop_monitor_event.set()
            if stop_thread.is_alive():
                stop_thread.join(timeout=2)
            sr.update_drawer_state(is_open=True, is_closed=False)
            if self.run_aborted:
                # Emit whatever optics were captured, labeled aborted; an
                # aborted run with no capture emits no event (#288).
                sr.emit_optics_readings(
                    str(optics_log),
                    run_timestamp=self.run_timestamp,
                    expected_lines=self._optics_expected_lines(),
                    aborted=True,
                )
                sr.change_screen("1")
            else:
                try:
                    os.makedirs("logs/results", exist_ok=True)
                    results_to_json( optics_log, results_json )
                    sr.mark_results_ready(results_json)
                except Exception as e:
                    logger.error("Failed to generate results: %s", e)
                graph_path = None
                try:
                    os.makedirs("logs/plots", exist_ok=True)
                    generate_optics_plot(optics_log, plot_path)
                    graph_path = f"/plots/{plot_filename}"
                except Exception as e:
                    logger.error("Failed to generate plot: %s", e)
                profile_name = self.thermal_profile.replace("profiles/", "")
                # Snapshot the operator's tube names once at completion so history
                # and the run_complete event carry the SAME labels captured with
                # this run, not two independent re-reads of the mutable global (#296).
                tube_names = sr.get_tube_names()
                sr.log_history(
                    profile_name, self.run_name, results_json, graph_path,
                    tube_names=tube_names,
                )
                sr.emit_run_complete(
                    self.run_name, profile_name, str(results_json),
                    run_timestamp=self.run_timestamp,
                    tube_names=tube_names,
                )
                # Capture the exact optics file just consumed onto the same
                # outbox, sharing run_timestamp (#288).
                sr.emit_optics_readings(
                    str(optics_log),
                    run_timestamp=self.run_timestamp,
                    expected_lines=self._optics_expected_lines(),
                    aborted=False,
                )
                sr.advance_run_name()
                sr.change_screen("3")


    def _optics_expected_lines(self) -> int:
        # Prefer the profile's planned pass count so an aborted run's
        # expected_lines is the intended total (complete=false, honest coverage);
        # fall back to the runtime count if the profile couldn't be parsed (#288).
        passes = self._planned_optics_passes or self._optics_pass_count
        if self.well_15: return expected_lines(passes, 21)
        else: return expected_lines(passes, READS_PER_CYCLE)

    def hw_deinitialize(self):
        self.meer.setTargetObjectTemperature ( 25.0 )
        self.meer.output_stage_enable ( 0 )

        self.lid_heater_stop_event.set()
        if hasattr(self, "lid_thread") and self.lid_thread.is_alive():
            self.lid_thread.join( timeout = 5 )

        # issue #157: pin a lid-thread leak to this run. thread_still_alive=True
        # or lid_live>0 here means the join above gave up and the worker leaked.
        still_alive = hasattr(self, "lid_thread") and self.lid_thread.is_alive()
        logger.info(
            "LID JOIN DONE run_index=%d thread_still_alive=%s lid_live=%d",
            getattr(self, "_run_index", -1), still_alive, lwm.live_count(),
        )


    def end( self ):
        #End of run
        # Returns True if the operator armed another run from the results
        # screen (issue #333), so the caller can start it directly instead of
        # falling back to ready() — which would silently eat the first press.
        if self.run_aborted:
            self.run_aborted = False
            sr.timer_control( "stop" )
            sr.timer_control( "reset" )
            sr.change_screen("1")
            return False
        sr.timer_control( "stop" )
        time.sleep(2)
        sr.timer_control( "reset" )
        sr.change_screen("3")
        # Drop any Run press that was latched during the run before we inspect
        # the button (#333 regression): /button/run keeps setting run_requested
        # mid-run, so a double-tap at run start would still be pending here and
        # arm a phantom next run. Clearing it now means only a fresh press on
        # the results screen arms the next run; the profile is preserved so that
        # single press still reuses it.
        sr.reset_run_request()
        profile, run_name = self.button_logic( state = "end" )
        if profile is not None:
            # Run pressed on the results screen: arm the next run here,
            # mirroring ready() (self.run_name / self.thermal_profile), so the
            # single press that got us out of button_logic actually runs.
            self.run_name = run_name
            self.thermal_profile = ("profiles/" + profile)
            return True
        return False

    def _monitor_stop_request(self, stop_event: Event, stop_monitor_event: Event) -> None:
        while not stop_monitor_event.is_set() and not stop_event.is_set():
            if sr.check_stop_request():
                logger.info("Stop request detected")
                stop_event.set()
                sr.reset_stop_request()
                return
            time.sleep(0.5)

    def button_logic(self, state = "ready"):
        include_run_complete_ack = state == "end"
        ret = sr.wait_for_button(include_run_complete_ack)
        
        if(state == "end"):
            screens = ["8","3","9","1"]  
            #8 test complete drawer open
            #3 test complete remove samples
            #9 test complete drawer close
            #1 ready to run
        elif(state == "ready"):
            screens = ["6","1","7","4"]    
            #6 Ready to run drawer open
            #1 Ready to run
            #7 Ready to run Drawer close selected
            #4 Ready to run No profile selected Try again...

        while True:
            run = ret.get("run_requested")
            profile = ret.get("profile")
            run_name = ret.get("run_name")
            drawer_open = ret.get("drawer_open_status")
            drawer_close = ret.get("drawer_close_status")
            exit_status = ret.get("exit_button_status")
            force_exit = ret.get("force_exit")
            run_complete_ack = ret.get("run_complete_ack")

            if( run is True and profile is not None ):
                break
            elif( drawer_open is True and drawer_close is False ):
                self.drawer.open()
                sr.update_drawer_state(is_open=True, is_closed=False)
                if state != "end":
                    sr.change_screen( screens[0] )
                    #time.sleep(5) #simulate drawer opening
                    sr.change_screen( screens[1] ) 
                ret = sr.wait_for_button(include_run_complete_ack)
            elif( drawer_open is False and drawer_close is True ): 
                self.drawer.read()
                sr.update_drawer_state(is_open=False, is_closed=True)
                if state != "end":
                    sr.change_screen( screens[2] )
                    #time.sleep(5) #simulate drawer close
                    sr.change_screen( screens[1] ) 
                ret = sr.wait_for_button(include_run_complete_ack)
            elif( run is True and profile is None ):
                sr.change_screen( screens[3] )
                if( state == "ready" ): 
                    ret = sr.wait_for_button(include_run_complete_ack)
                elif( state == "end" ):
                    break
            elif force_exit:
                sr.change_screen("-4")
                time.sleep(3)
                self._exit_kiosk()
                sr.change_screen(screens[1])
                ret = sr.wait_for_button(include_run_complete_ack)
            elif( exit_status is True ):
                sr.change_screen("-5")
                ret = sr.wait_for_button(include_run_complete_ack)
                if(ret.get("exit_button_status")):
                    sr.change_screen("-4")
                    time.sleep(3)
                    self._exit_kiosk()
                    sr.change_screen(screens[1])
                    ret = sr.wait_for_button(include_run_complete_ack)
                else:
                    sr.change_screen( screens[1] )
                    ret = sr.wait_for_button(include_run_complete_ack)
            elif run_complete_ack and state == "end":
                sr.change_screen("1")
                sr.reset_run_complete_ack()
                ret = sr.wait_for_button(include_run_complete_ack)

        return profile, run_name

    def _exit_kiosk(self) -> None:
        """Ask the host kiosk-control service to kill Chromium.

        Tries the HTTP API first (works in both Docker and native deployments
        because kiosk-control always listens on 127.0.0.1:9191 on the host).
        Falls back to running exit_kiosk.sh as a subprocess in case the service
        is not yet running.
        """
        kiosk_control_url = os.getenv("KIOSK_CONTROL_URL", "http://127.0.0.1:9191")
        try:
            resp = _requests.post(f"{kiosk_control_url}/exit-kiosk", timeout=5)
            if resp.ok:
                logger.info("kiosk-control: exit-kiosk OK")
                return
            logger.warning("kiosk-control returned %s", resp.status_code)
        except Exception as e:
            logger.warning("kiosk-control unreachable, falling back to script: %s", e)

        # Fallback: run the shell script directly
        base_dir = Path(get_src_basedir())
        exit_script = base_dir / "exit_kiosk.sh"
        subprocess.run([str(exit_script)], check=False)

    def _safe_name(self, value: str | None) -> str:
        if not value:
            return "run"
        return re.sub(r"[^a-z0-9]+", "_", value.lower()).strip("_") or "run"
