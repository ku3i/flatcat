#include "flatcat_udp.hpp"

GlobalFlag do_quit;

namespace constants {
    const unsigned us_per_sec = 1000*1000;
    const unsigned update_rate_Hz = 100;
    const unsigned timeframe_us = us_per_sec/update_rate_Hz;
}

void
signal_terminate_handler(int signum)
{
    sts_msg("Got a SIGINT(%d) from user\n", signum);
    do_quit.enable();
}

inline bool starts_with(std::string msg, const char c_str[]) { return (msg.compare(0, strlen(c_str), c_str) == 0); }

template <typename T>
void parse_command(T& result, std::string const& msg, const char* keystr) {
    T value;
    if (1 == sscanf(msg.c_str(), keystr, &value)) {
        result = static_cast<T>(value);
        //dbg_msg("'%s' command received.", keystr);
    } else wrn_msg("'%s' command broken.", keystr);
}


template <typename Vector_t>
void parse_midi_channel(Vector_t& vec, std::string const& msg) {
    unsigned idx;
    float value;
    if (2 == sscanf(msg.c_str(), "MDI%u=%f", &idx, &value) and idx < vec.size()) {
        vec.at(idx) = value;
        //dbg_msg("MIDI %02u = %+5.2f", idx, value);
    } else wrn_msg("Midi command broken.", msg);
}

/* simple command parser, replace if there is some time (TM) */
void MainApplication::handle_tcp_commands(std::string const& msg)
{
    if (starts_with(msg, "ENA")) { parse_command(control.enabled  , msg, "ENA=%u"); return; }

    if (starts_with(msg, "PAR")) {
        parse_command(control.parameter_id, msg, "PAR=%u");
        if (control.parameter_id < control.parameter_set.size()) {
            sts_msg("Set control parameter set number %u.", control.parameter_id);
            //control.mixed_jointcontrol.set_active(control.parameter_id);
            control.jointcontrol.set_control_parameter(control.parameter_set.get(control.parameter_id));
            control.modulate = (control.parameter_id == 0) ? 0.f : 1.f;
        } else wrn_msg("No control parameters for ID = %u", control.parameter_id);
        return;
    }

    if (starts_with(msg, "AMP")) { parse_command(control.amplitude, msg, "AMP=%f"); return; }
    if (starts_with(msg, "MOD")) { parse_command(control.modulate , msg, "MOD=%f"); return; }
    if (starts_with(msg, "ING")) { parse_command(control.inputgain, msg, "ING=%f"); return; }

    if (starts_with(msg, "CTL")) { parse_command(control.mode     , msg, "CTL=%u"); return; }
    if (starts_with(msg, "POS")) { parse_command(control.usr_pos  , msg, "POS=%u"); return; }

    if (starts_with(msg, "MDI")) { parse_midi_channel(control.usr_params, msg); return; }

    if (msg == "RST")   { sts_msg("Reset motor statistics."); flatcat.reset_motor_statistics(); return; }
    if (msg == "HELLO") { sts_msg("client says hello"   ); return; }

    if (msg == "CEN")   { calibrate.trigger(1); return; }
    if (msg == "CAB")   { calibrate.trigger(2); return; }
    if (msg == "CID")   { calibrate.toggle_index(); return; }

    dbg_msg("unknown msg: %s", msg.c_str());
}

int main(int argc, char* argv[])
{
    sts_msg("Initializing Flatcat <3");
    srand((unsigned) time(NULL));
    signal(SIGINT, signal_terminate_handler);

    MainApplication app(argc, argv, do_quit);

    std::thread tcp_thread(&MainApplication::tcp_serv_loop, &app);
    std::thread udp_thread(&MainApplication::udp_send_loop, &app);

    Stopwatch watch;

    const bool verbose = (argc == 2 && strcmp (argv[1],"-v") == 0) ? true : false;
    sts_msg("Verbose mode: %s", (verbose)? "on" : "off");

    sts_msg("Starting main loop.");
    watch.reset();
    while(!do_quit.status())
    {
        app.execute_cycle();

        if (verbose)
            sts_msg("%05.2f ms", watch.get_time_passed_us()/1000.0);
    }
    sts_msg("Waiting for UDP communication thread to join.");
    udp_thread.join();
    sts_msg("Waiting for TCP communication thread to join.");
    tcp_thread.join();
    app.finish();
    sts_msg("____\nDONE.");
    return 0;
}

