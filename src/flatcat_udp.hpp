#ifndef FLATCAT_UDP_HPP
#define FLATCAT_UDP_HPP

#include <array>
#include <thread>
#include <signal.h>

#include <common/log_messages.h>
#include <common/globalflag.h>
#include <common/stopwatch.h>
#include <common/basic.h>
#include <common/modules.h>

#include <control/jointcontrol.h>
#include <control/control_vector.h>

#include <flatcat_robot.hpp>
#include <flatcat_control.hpp>
#include <flatcat_settings.hpp>
//#include <spinalcord.hpp> //TODO replace with motorcord for timing information

#include <common/udp.hpp>
#include <common/socket_server.h>


/* calibration procedure:

    + press 'c' (calibration enable, index = 0)
    + one after another,
        move all joints to their desired movement limits l1 to l0
        compare with defined model limits (180 = c1 - c0)

        o1 = c1 - l1
        o0 = c0 - l0

        offset = (o1 + o0) / 2
        leave calibration mode with 'c'
        save and confirm with 'J' abort with 'N'
*/

namespace supreme {

class FlatcatCalibration
{
    robots::Jointvector_t& joints;
    supreme::motorcord& motors;
    file_io::CSV_File<float> csvfile;

    enum CalibrationState { done, init, select, calib, save, finish } state = done;

    unsigned index = 0;
    unsigned trig = 0;

    struct CalibVal_t {
        float lmin = +1.f;
        float lmax = -1.f;
        float offset_diff = .0f;
        float offset_base = .0f;
    } val;

    unsigned steps = 0;

public:
    FlatcatCalibration(supreme::FlatcatRobot& robot, std::string const& filename)
    : joints(robot.set_joints())
    , motors(robot.set_motors())
    , csvfile(filename, /*rows=*/joints.size(), /*cols=*/1)
    , val()
    {
        if (!csvfile.read())
            wrn_msg("Cannot read from csv file: %s", csvfile.get_filename());
        else
        {
            for (unsigned i = 0; i < motors.size(); ++i)
            {
                float val;
                csvfile.get_line(i, val);
                motors[i].set_offset(val);
                sts_msg("joint %02u offset = %+6.3f (%s)",i, val, joints.at(i).name.c_str());
            }
            sts_msg("Read and applied motor calibration data.");
        }

    }

    void show_usage(void) {
        sts_msg(
            "________________\n"
            "CALIBRATION MODE\n"
            "usage: \n"
            " v: increment joint index\n"
            " c: confirm/save\n"
            " b: discard/exit\n"
        );
    }

    void reset(void) { val = CalibVal_t{}; }

    void execute_cycle(void) {

        switch(state) {
        case done: if (trig==1) state = init; break;

        case init:
            show_usage();
            reset();
            state = select;
            break;

        case select:
            if (trig==1) {
                sts_msg("Move leg to the limits and confirm or discard.");
                state = calib;
            }
            else if (trig==2) state = finish;
            break;

        case calib:
            capture();
            if (trig==1) state = save;
            else if (trig==2) { sts_msg("\nDiscarded."); state = finish; }
            break;

        case save:
            apply_offset_and_save_to_file();
            state = finish;
            break;

        case finish:
        default:
            sts_msg("____\nDONE"); state = done;
            break;
        }

        trig = 0;
    }

    void capture(void) {
        steps = (steps+1)%10;
        auto const& j = joints[index];
        const float pos = j.s_ang;
        val.lmin = std::min(pos, val.lmin);
        val.lmax = std::max(pos, val.lmax);
        val.offset_diff = ((j.limit_hi - val.lmax) + (j.limit_lo - val.lmin))/2;
        if (steps == 0) {
            printf("\rj:%u pos:%+5.2f min:%+5.2f(%+5.2f) max:%+5.2f(%+5.2f) ofs:%+6.3f "
                  , index, pos, val.lmin, j.limit_lo, val.lmax, j.limit_hi, val.offset_diff);
            fflush(stdout);
        }
    }

    void trigger(unsigned t) { trig = t; }

    void toggle_index(void) {
        if (state != select) return;
        index = (index+1) % joints.size();
        sts_msg("CALIBRATION INDEX = %u (%s)", index, joints[index].name.c_str());
    }

    bool is_enabled(void) const { return state != done; }

    void apply_offset_and_save_to_file(void) {
        auto& m = motors[index];
        m.add_offset(val.offset_diff);
        csvfile.set_line(index, m.get_offset());
        csvfile.write();
        sts_msg("\nSaved.");
    }
};

} /* namespace supreme */

class MainApplication
{
public:
    MainApplication(int argc, char** argv, GlobalFlag const& do_quit)
    : do_quit(do_quit)
    , settings(argc, argv)
    , flatcat(settings)
    , control(flatcat, settings)
    , calibrate(flatcat, "calib.csv")
    , command_server(7332 /*TODO command port*/)
    , udp_sender(settings.group, settings.port)
    , sendbuffer()
    {
        sts_msg("____\nDONE initializing Flatcat controller.");
    }

    bool execute_cycle() {

        if (calibrate.is_enabled()) {
            control.amplitude = .0f;
            control.enabled = false; // assure robot motors turned off
        }
        calibrate.execute_cycle();

        flatcat.set_voltage_amplitude(control.amplitude);
        flatcat.set_enable(control.enabled);

        flatcat.execute_cycle();
        control.execute_cycle();

        fill_sendbuffer();
        udp_sender.set_buffer(sendbuffer.get(), sendbuffer.size());

        ++cycles;

        return true; // not used
    }

    void finish() {/*TODO implement*/};

    void udp_send_loop(void)
    {
        while(!do_quit.status())
        {
            if (udp_sender.data_ready()) {
                udp_sender.transmit();
            }
            else usleep(100); // reduce polling
        }

    }

    void tcp_serv_loop(void)
    {
        std::string msg = ""; //TODO handle this thread-safe, hahahaha!

        sts_msg("Starting TCP command server, waiting for incoming connections.");
        while(!do_quit.status())
        {
            while(!command_server.open_connection()) {
                if (do_quit.status())
                    break;
            }

            if (!do_quit.status())
                udp_sender.change_destination(command_server.get_current_client_address());

            while(!do_quit.status()) {
                msg = command_server.get_next_line();
                if (msg == "EXIT") {
                    sts_msg("Client requested to close connection.");
                    break;
                } else
                    handle_tcp_commands(msg);
            }

            command_server.close_connection();
        }
    }


    void fill_sendbuffer(void) {
        sendbuffer.reset();

        sendbuffer.add(cycles);

        /* N motors */
        for (unsigned i = 0; i < flatcat.get_motors().size(); ++i)
        {
            auto const& m = flatcat.get_motors()[i];
            auto const& d = m.get_data();
            sendbuffer
            .add(m.get_id()         )
            .add(d.position         )
            .add(d.last_p           )
            .add(d.velocity         )
            .add(d.current          )
            .add(d.voltage_supply   )
            .add(d.output_voltage   )
//TODO            .add(d.voltage_backemf  )
//TODO            .add(d.last_output      )
            .add(d.temperature      )
//TODO            .add(d.is_connected     )
            //.add(m.connection_losses)
            //.add(m.dir              )
            //.add(m.scale            )
            //.add(m.offset           );
            ;
        }

        /**TODO
        auto const& t = flatcat.get_spinalcord_timing();
        sendbuffer
        .add(t.mean                    )
        .add(t.maxv                    )
        .add(t.syncfaults              )
        .add(t.board_dropouts          )
        .add(t.transparent_errors      )
        .add(t.transparent_packets_recv)
        .add(t.collected_ids           );
        */

        auto const& c = control;
        sendbuffer
        .add(c.enabled  )
        .add(c.usr_pos  )
        .add(c.amplitude)
        .add(c.modulate )
        .add(c.inputgain)
        .add(c.cur_mode );

        sendbuffer.add_checksum();
    }


    void handle_tcp_commands(std::string const& msg);

    void calibration_procedure(void);

private:

    GlobalFlag const&           do_quit;
    supreme::FlatcatSettings    settings;
    supreme::FlatcatRobot       flatcat;
    supreme::FlatcatControl     control;
    supreme::FlatcatCalibration calibrate;

    network::Socket_Server     command_server;

    network::UDPSender <113>   udp_sender;
    network::Sendbuffer<113>   sendbuffer;



    uint64_t cycles = 0;
};


#endif /* FLATCAT_UDP_HPP */
