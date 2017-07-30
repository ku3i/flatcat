/*---------------------------------+
 | Matthias Kubisch                |
 | kubisch@informatik.hu-berlin.de |
 | July 2017                       |
 +---------------------------------*/

#include <vector>
#include <cassert>
#include <string>
#include <memory>
#include <common/log_messages.h>
#include "RtMidi.h"

/*
    g++ -Wall -D__LINUX_ALSA__ -o test test.cpp RtMidi.cpp -lasound -lpthread
*/

class MidiIn
{
    static constexpr std::size_t max_channel = 256;
    static constexpr double      default_val = 0.0;

    std::unique_ptr<RtMidiIn>  midi_ptr;
    std::vector<unsigned char> message;
    std::vector<double>        data;
    std::string                interface_name;
    bool                       init_success;

public:
    MidiIn(unsigned port)
    : midi_ptr(nullptr)
    , message()
    , data(max_channel, default_val)
    , interface_name()
    , init_success(false)
    {
        assert(data.size() == max_channel);
        try {
            midi_ptr = std::unique_ptr<RtMidiIn>(new RtMidiIn());
        }
        catch (RtMidiError &error) {
            error.printMessage();
            err_msg(__FILE__,__LINE__,"Error while initializing MIDI system.");
        }

        if (port < midi_ptr->getPortCount())
        {
            try {
                midi_ptr->openPort(port);
            }
            catch (RtMidiError &error) {
                error.printMessage();
                err_msg(__FILE__,__LINE__,"Error while opening MIDI port: %u", port);
            }
        } else {
            wrn_msg("No MIDI interface initialized. No device on MIDI port: %u", port);
            return;
        }

        midi_ptr->ignoreTypes( false, false, false );

        init_success = true;
        interface_name = midi_ptr->getPortName(port);
        sts_msg("MIDI Interface '%s' successfully initialized.", interface_name.c_str());
    }

    bool fetch() {
        if (not init_success) return false;

        try {
            const double dt = midi_ptr->getMessage( &message );
            const std::size_t num_bytes = message.size();

            if (num_bytes >= 2) {
                const unsigned char channel = message[1];
                data[channel] = static_cast<double>(message[2] / 127.0);
                dbg_msg("MIDI(%03u): ch=%03u val=%4.2f dt=%4.2f", message[0], message[1], data[channel], dt);
            }

        }
        catch (RtMidiError &error) {
            error.printMessage();
            return false;
        }

        return true;
    }

    double operator[] (std::size_t index) const { return data.at(index); }

};


