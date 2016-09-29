#ifndef GUI_H
#define GUI_H

#include <vector>
#include <memory>
#include <thread>
#include <locale.h>
#include <gtk/gtk.h>
#include <common/globalflag.h>
#include <common/log_messages.h>


extern float progress_val1;
extern float progress_val2;

extern GlobalFlag do_pause;

gboolean update_progressbar1(gpointer data);
gboolean update_progressbar2(gpointer data);

int thread_gtk_main(void);

void on_button_start_clicked(GtkObject *object, gpointer user_data);

class vertical_scaler
{
public:
    vertical_scaler(double value_min, double value_max, double value_default, double increment)
    : adjustment((GtkAdjustment*) gtk_adjustment_new(value_default, value_min, value_max, increment, 1.0, 0.0))
    , vscale(gtk_vscale_new(adjustment))
    {
        gtk_scale_set_digits(GTK_SCALE(vscale), 1); //TODO num digits
        gtk_scale_set_value_pos(GTK_SCALE(vscale), GTK_POS_BOTTOM);
        gtk_widget_show(vscale);
    }

    GtkWidget* get(void) { return vscale; }

private:
    GtkAdjustment *adjustment;
    GtkWidget     *vscale;
};

class gui_interface
{
public:
    virtual ~gui_interface() {};
};

class no_gui : public gui_interface
{
public:
    no_gui() { sts_msg("No GUI loaded."); }
};

class GTK_gui : public gui_interface
{
    GTK_gui(const GTK_gui& other) = delete;      // non construction-copyable
    GTK_gui& operator=( const GTK_gui&) = delete; // non copyable

public:
    GTK_gui();

    ~GTK_gui()
    {
        dbg_msg("closing GTK main thread");
        gtk_main_quit();
        main_gtk->join();
    }

private:
    const unsigned int num_vscale;
    const bool init_result;

    GtkWidget *window;
    GtkWidget *table;
    GtkWidget *label;
    GtkWidget *progressbar1;
    GtkWidget *progressbar2;
    GtkWidget *button_start;

    std::vector<vertical_scaler> multiscale;
    std::unique_ptr<std::thread> main_gtk;
};

class GUI_Starter
{
    std::unique_ptr<gui_interface> gui;

    GUI_Starter(const GUI_Starter& other) = delete;      // non construction-copyable
    GUI_Starter& operator=( const GUI_Starter&) = delete; // non copyable

public:
    GUI_Starter(bool visuals_enabled = true) : gui(visuals_enabled ? (gui_interface*) new GTK_gui() : (gui_interface*) new no_gui()) {}
};

#endif /*GUI_H*/
