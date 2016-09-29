/* gui.cpp */
#include "./gui.h"

float progress_val1;
float progress_val2;

extern GlobalFlag do_pause;


gboolean
update_progressbar1(gpointer data)
{
    gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(data), progress_val1);
    return 1;
}

gboolean
update_progressbar2(gpointer data)
{
    gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(data), progress_val2);
    return 1;
}

int
thread_gtk_main(void)
{
    gtk_main();
    return 0;
}

void
on_button_start_clicked(GtkObject *object, gpointer user_data)
{
    do_pause.toggle();

    if (do_pause.status()) {
        gtk_button_set_label(GTK_BUTTON(object), "Start");
        sts_msg("Paused.");
    }
    else {
        gtk_button_set_label(GTK_BUTTON(object), "Pause");
        sts_msg("Continuing.");
    }
}

GTK_gui::GTK_gui(void)
: num_vscale(10)
, init_result(gtk_init_check(0, NULL))
, window(gtk_window_new(GTK_WINDOW_TOPLEVEL))
, table(gtk_table_new(6, num_vscale, TRUE))
, label(gtk_label_new("Progress Bar Example"))
, progressbar1(gtk_progress_bar_new())
, progressbar2(gtk_progress_bar_new())
, button_start(gtk_button_new_with_label("Start"))
, multiscale()
, main_gtk()
{

    multiscale.reserve(num_vscale);
    for (unsigned int i = 0; i < num_vscale; ++i)
        multiscale.emplace_back(0.0, 100.0, 50.0, 0.1);

    sts_msg("Starting GTK window.");
    if (!init_result) {
        wrn_msg("Starting without GTK-Window.");
        return;
    }

    setlocale(LC_NUMERIC, "C");

    gtk_container_add(GTK_CONTAINER(window), table);
    gtk_table_set_row_spacing(GTK_TABLE(table), 3, 100);
    //gtk_table_set_row_spacings(GTK_TABLE(table), 100);

    /* set positions */
    gtk_table_attach_defaults(GTK_TABLE(table), label       , 0, num_vscale, 0, 1);
    gtk_table_attach_defaults(GTK_TABLE(table), progressbar1, 0, num_vscale, 1, 2);
    gtk_table_attach_defaults(GTK_TABLE(table), progressbar2, 0, num_vscale, 2, 3);

    for (unsigned int i = 0; i < num_vscale; ++i)
        gtk_table_attach_defaults(GTK_TABLE(table), multiscale[i].get(), 0+i, 1+i, 3, 5);

    gtk_table_attach_defaults(GTK_TABLE(table), button_start, 0, 1, 5, 6);

    /* Set the timeout to handle automatic updating of the progress bar */
    g_timeout_add(1000, update_progressbar1, progressbar1);
    g_timeout_add(500,  update_progressbar2, progressbar2);

    /* signals */
    g_signal_connect(GTK_OBJECT (button_start), "clicked", GTK_SIGNAL_FUNC(on_button_start_clicked), NULL);
    g_signal_connect(window, "delete_event", G_CALLBACK(gtk_window_iconify), NULL); // minimize window

    /* show */
    gtk_widget_show(button_start);
    gtk_widget_show(progressbar1);
    gtk_widget_show(progressbar2);
    gtk_widget_show(label);
    gtk_widget_show(table);
    gtk_widget_show(window);

    main_gtk = std::unique_ptr<std::thread>(new std::thread(thread_gtk_main));
}
