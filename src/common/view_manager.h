#ifndef VIEW_MANAGER_H_INCLUDED
#define VIEW_MANAGER_H_INCLUDED

#include <common/log_messages.h>

class View_Manager {

    const unsigned num_views;
          unsigned view_id;

public:
    View_Manager(const std::size_t num_views) : num_views(num_views), view_id(0) {}

    unsigned get() const { return view_id; }
    void set(unsigned v)       { if (v < num_views) view_id = v; else wrn_msg("Could not set view: %u", v); }

    void key_pressed(SDL_Keysym const& keysym)
    {
        switch (keysym.sym)
        {
            case SDLK_v : ++view_id; if (view_id >= num_views) view_id = 0; break;
            default     : return;
        }
    }

};

#endif // VIEW_MANAGER_H_INCLUDED
