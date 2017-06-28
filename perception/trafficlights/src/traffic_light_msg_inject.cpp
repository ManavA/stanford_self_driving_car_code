#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <applanix_interface.h>
#include <localize_messages.h>
#include <lltransform.h>
#include <rndf.h>
#include <gui2D.h>
#include <transform.h>
#include <passat_constants.h>
#include <cmath>

#include "../../interface/traffic_lights/traffic_light_messages.h"
#include <gtk/gtk.h>
#include <stdio.h>

using namespace dgc;
using namespace std;

//GLOBALS
param_struct_t param;
TrafficLightState light;

IpcInterface *ipc;

static void update_callback( GtkWidget *widget, __attribute__ ((unused))  gpointer   data );
static void startStop_callback( __attribute__ ((unused)) GtkWidget *widget, __attribute__ ((unused)) gpointer   data );


static char *GrayBox[] =
{
	/* width height num_colors chars_per_pixel */
	"    25    10        1            1",
	/* colors */
	". c #e6e6e6",
	/* pixels */
	".........................",
	".........................",
	".........................",
	".........................",
	".........................",
	".........................",
	".........................",
	".........................",
	".........................",
	"........................."
};
static char *RedBox[] =
{
	"    25    10        1            1",
	". c #dd0000",
	".........................",
	".........................",
	".........................",
	".........................",
	".........................",
	".........................",
	".........................",
	".........................",
	".........................",
	"........................."
};
static char *GreenBox[] =
{
	"    25    10        1            1",
	". c #00dd00",
	".........................",
	".........................",
	".........................",
	".........................",
	".........................",
	".........................",
	".........................",
	".........................",
	".........................",
	"........................."
};

static char *YellowBox[] =
{
	"    25    10        1            1",
	". c #ffff00",
	".........................",
	".........................",
	".........................",
	".........................",
	".........................",
	".........................",
	".........................",
	".........................",
	".........................",
	"........................."
};

static GdkColor colorGray;
static GdkColor colorRed;

//global state variables
bool sending = false;
bool rg_switch = false;
bool gy_switch = false;
char currState = 'g';

// Global variables
GtkWidget *redBox;
GtkWidget *greenBox;
GtkWidget *grayBox;
GtkWidget *yellowBox;
//for entering coordinates
GtkWidget *entry_ID;
GtkWidget *id, *status;


// Callback:  The data passed to this function is printed to stdout.
static void callback1( __attribute__ ((unused)) GtkWidget *widget,
                       __attribute__ ((unused)) gpointer   data )
{

	gtk_widget_hide(grayBox);
	gtk_widget_hide(redBox);
	gtk_widget_hide(yellowBox);
	gtk_widget_show(greenBox);
	if (currState == 'r')
	{
		rg_switch = true;
	}
	currState = 'g';
}

static void callback2( __attribute__ ((unused)) GtkWidget *widget,
                       __attribute__ ((unused)) gpointer   data )
{

	gtk_widget_hide(grayBox);
	gtk_widget_hide(greenBox);
	gtk_widget_hide(yellowBox);
	gtk_widget_show(redBox);
	currState = 'r';
}

static void callback3( __attribute__ ((unused)) GtkWidget *widget,
                       __attribute__ ((unused))  gpointer   data )
{

	gtk_widget_hide(grayBox);
	gtk_widget_hide(greenBox);
	gtk_widget_hide(redBox);
	gtk_widget_show(yellowBox);
	if (currState == 'g')
	{
		gy_switch = true;
	}
	currState = 'y';
}

/* Callback to close window */
static gboolean delete_event( __attribute__ ((unused)) GtkWidget *widget,
                              __attribute__ ((unused)) GdkEvent  *event,
                              __attribute__ ((unused)) gpointer   data )
{
	g_print("delete_event\n");
	gtk_main_quit ();
	return FALSE;
}

void GUI_main( int argc, char *argv[])
{

	gdk_color_parse("gray", &colorGray);
	gdk_color_parse("red", &colorRed);

	GtkWidget *window;
	GdkPixmap *_icon_red;
	GdkPixmap *_icon_green;
	GdkPixmap *_icon_gray;
	GdkBitmap *_icon_mask_red;
	GdkBitmap *_icon_mask_green;
	GdkBitmap *_icon_mask_gray;
	GtkStyle  *_style_red;
	GtkStyle  *_style_green;
	GtkStyle  *_style_gray;
	GtkWidget *button1;
	GtkWidget *button2;
	GtkWidget *button3;
	GtkWidget *startStop, *update;
	GtkWidget *hbox1, *hbox2, *hbox3;
	GtkWidget *entry_table, *whole_table;

	// This is called in all GTK applications. Arguments are parsed
	// from the command line and are returned to the application.
	gtk_init (&argc, &argv);

	window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
	gtk_window_set_title (GTK_WINDOW (window), "(Fake) Traffic Light Message Injector");

	// Here we just set a handler for delete_event that immediately exits GTK.
	g_signal_connect (G_OBJECT (window), "delete_event",
	                  G_CALLBACK (delete_event), NULL);

	gtk_container_set_border_width (GTK_CONTAINER (window), 10);
	gtk_widget_show (window);  // MUST put this here and NOT at end.

	//setup the layout framework
	hbox1 = gtk_hbox_new(FALSE, 0);
	hbox2 = gtk_hbox_new(FALSE, 0);
	hbox3 = gtk_hbox_new(FALSE, 0);
	whole_table = gtk_table_new(4, 1, FALSE);
	entry_table = gtk_table_new(4, 2, FALSE);
	gtk_table_attach( GTK_TABLE(whole_table), hbox1, 0, 1, 0, 1, GTK_FILL, GTK_FILL, 1, 1);
	gtk_table_attach( GTK_TABLE(whole_table), entry_table, 0, 1, 1, 2, GTK_FILL, GTK_FILL, 1, 1);
	gtk_table_attach( GTK_TABLE(whole_table), hbox2, 0, 1, 2, 3, GTK_FILL, GTK_FILL, 1, 1);
	gtk_table_attach( GTK_TABLE(whole_table), hbox3, 0, 1, 3, 4, GTK_FILL, GTK_FILL, 1, 1);
	gtk_container_add (GTK_CONTAINER (window), whole_table);


	//status label
	status =  gtk_label_new("not sending...");
	gtk_box_pack_start(GTK_BOX(hbox1), status, TRUE, TRUE, 0);
	gtk_widget_show (status);


	// ------------------------------------------------------------------------
	//                     Gray
	_style_gray = gtk_widget_get_style( window );
	_icon_gray = gdk_pixmap_create_from_xpm_d(GTK_WIDGET(window)->window,
	             &_icon_mask_gray,
	             &_style_gray->bg[GTK_STATE_NORMAL],
	             GrayBox);
	grayBox = gtk_pixmap_new(_icon_gray, _icon_mask_gray);
	g_object_unref(_icon_gray);
	g_object_unref(_icon_mask_gray);
	gtk_widget_show(grayBox);

	//                     Green
	_style_green = gtk_widget_get_style( window );
	_icon_green = gdk_pixmap_create_from_xpm_d(GTK_WIDGET(window)->window,
	              &_icon_mask_green,
	              &_style_green->bg[GTK_STATE_NORMAL],
	              GreenBox);
	greenBox = gtk_pixmap_new(_icon_green, _icon_mask_green);
	g_object_unref(_icon_green);
	g_object_unref(_icon_mask_green);

	//                     Red
	_style_red = gtk_widget_get_style( window );
	_icon_red = gdk_pixmap_create_from_xpm_d(GTK_WIDGET(window)->window,
	            &_icon_mask_red,
	            &_style_red->bg[GTK_STATE_NORMAL],
	            RedBox);
	redBox = gtk_pixmap_new(_icon_red, _icon_mask_red);
	g_object_unref(_icon_red);
	g_object_unref(_icon_mask_red);

	//                     yellow
	_style_red = gtk_widget_get_style( window );
	_icon_red = gdk_pixmap_create_from_xpm_d(GTK_WIDGET(window)->window,
	            &_icon_mask_red,
	            &_style_red->bg[GTK_STATE_NORMAL],
	            YellowBox);
	yellowBox = gtk_pixmap_new(_icon_red, _icon_mask_red);
	g_object_unref(_icon_red);
	g_object_unref(_icon_mask_red);



	// ------------------------------------------------------------------------

	// Attach images to the same place
	gtk_box_pack_start (GTK_BOX(hbox1), grayBox, TRUE, TRUE, 0);
	gtk_box_pack_start(GTK_BOX(hbox1), greenBox, TRUE, TRUE, 0);
	gtk_box_pack_start (GTK_BOX(hbox1), redBox, TRUE, TRUE, 0);
	gtk_box_pack_start (GTK_BOX(hbox1), yellowBox, TRUE, TRUE, 0);

	// Position both images in the same location
	gtk_misc_set_alignment(GTK_MISC(grayBox), 1, 0.5);
	gtk_misc_set_alignment(GTK_MISC(greenBox), 1, 0.5);
	gtk_misc_set_alignment(GTK_MISC(redBox), 1, 0.5);
	gtk_misc_set_alignment(GTK_MISC(yellowBox), 1, 0.5);

	// ------------------------------------------------------------------------

	button1 = gtk_button_new_with_label("  Green  ");
	g_signal_connect (G_OBJECT (button1), "clicked",
	                  G_CALLBACK (callback1), (gpointer) "button1");
	gtk_box_pack_start(GTK_BOX(hbox1), button1, TRUE, TRUE, 0);
	gtk_widget_show (button1);

	// ------------------------------------------------------------------------

	button2 = gtk_button_new_with_label("   Red   ");
	g_signal_connect (G_OBJECT (button2), "clicked",
	                  G_CALLBACK (callback2), (gpointer) "button2");
	gtk_box_pack_start(GTK_BOX(hbox1), button2, TRUE, TRUE, 0);
	gtk_widget_show (button2);

	// ------------------------------------------------------------------------

	button3 = gtk_button_new_with_label("   Yellow   ");
	g_signal_connect (G_OBJECT (button3), "clicked",
	                  G_CALLBACK (callback3), (gpointer) "button3");
	gtk_box_pack_start(GTK_BOX(hbox1), button3, TRUE, TRUE, 0);
	gtk_widget_show (button3);

	// ------------------------------------------------------------------------

	id =  gtk_label_new("traffic light ID: ");
	gtk_table_attach( GTK_TABLE(entry_table), id, 0, 1, 0, 1, GTK_FILL, GTK_FILL, 0, 0);
	gtk_widget_show (id);

	entry_ID = gtk_entry_new ();
	gtk_entry_set_text(GTK_ENTRY(entry_ID), "0");
	gtk_entry_set_max_length (GTK_ENTRY (entry_ID), 50);
	/*g_signal_connect (G_OBJECT (entry), "activate",
		      G_CALLBACK (enter_callback),
		      (gpointer) entry);*/
	gtk_table_attach( GTK_TABLE(entry_table), entry_ID, 1, 2, 0, 1, GTK_FILL, GTK_FILL, 0, 0);
	gtk_widget_show (entry_ID);

	//-------------------------------------------

	update = gtk_button_new_with_label(" update ID");
	g_signal_connect (G_OBJECT (button1), "clicked",
	                  G_CALLBACK (update_callback), (gpointer) "update");
	gtk_box_pack_start(GTK_BOX(hbox3), update, TRUE, TRUE, 0);
	gtk_widget_show (update);

	startStop = gtk_button_new_with_label("  Start/Stop sending messages");
	g_signal_connect (G_OBJECT (startStop), "clicked",
	                  G_CALLBACK (startStop_callback), (gpointer) "startStop");
	gtk_box_pack_start(GTK_BOX(hbox3), startStop, TRUE, TRUE, 0);
	gtk_widget_show (startStop);


	//

	gtk_widget_show(hbox1);
	gtk_widget_show(hbox2);
	gtk_widget_show(hbox3);
	gtk_widget_show(whole_table);
	gtk_widget_show(entry_table);

	// Rest in gtk_main and wait for user input.
	gtk_main ();

}

//METHODS FOR REGISTERING AND PUBLISHING TrafficLightState status MESSAGES:

void dgc_TrafficLightState_register_ipc_messages()
{
	int err;

	err = ipc->DefineMessage(TrafficLightStateMsgID);
	TestIpcExit(err, "Could not define", TrafficLightStateMsgID);

}

void dgc_TrafficLightState_publish_status_message(TrafficLightState* message)
{
	int err;

	err = ipc->Publish(TrafficLightStateMsgID, (TrafficLightState *)message);
	TestIpcExit(err, "Could not publish", TrafficLightStateMsgID);
}


//send thread callback
void * sendMsgs( void *ptr)
{
	bool *status = (bool*)ptr;

	light.state_arrow = 'n';
	const gchar* buffer = gtk_entry_get_text(GTK_ENTRY(entry_ID));
	light.traffic_light_id = atoi(buffer);
	light.timestamp_rg_switch = 0;
	light.timestamp_gy_switch = 0;
	light.u = 500;
	light.v = 500;
	light.confidence = 1.0;
	double time = 0;
	while (status)
	{
		time = dgc_get_time();
		light.state = currState;
		if (rg_switch)
		{
			rg_switch = false;
			light.timestamp_rg_switch = time;
		}
		if (gy_switch)
		{
			gy_switch = false;
			light.timestamp_gy_switch = time;
		}
		light.timestamp = time;
		dgc_TrafficLightState_publish_status_message(&light);
		usleep(100000);
	}
	return 0;
}


//GUI CALLBACK FUNCTIONS:
static void update_callback( __attribute__ ((unused))GtkWidget *widget, __attribute__ ((unused))  gpointer   data )
{
	const gchar* buffer = gtk_entry_get_text(GTK_ENTRY(entry_ID));
	light.traffic_light_id = atoi(buffer);

}

static void startStop_callback( __attribute__ ((unused)) GtkWidget *widget, __attribute__ ((unused)) gpointer   data )
{
	sending = !sending;
	if (sending)
	{
		gtk_label_set_text(GTK_LABEL(status),"    sending...");
		pthread_t sendingThread;
		pthread_create(&sendingThread, NULL, sendMsgs, &sending);
	}
	else
	{
		gtk_label_set_text(GTK_LABEL(status),"not sending...");
	}

}


//THREADS:
void * run_GUI_function( void *ptr)
{
	param_struct_p param = (param_struct_t*)ptr;
	GUI_main(param->argc, param->argv);
	return 0;

}


//Initialization and Shutdown:

static void shutdown_module(int x)
{
	if (x == SIGINT)
	{
		fprintf(stderr, "\nTRAFFIC_LIGHT_MSG_INJECT: Caught SIGINT. exiting.\n");
		fflush(stderr);
		ipc->Disconnect();
		exit(1);
	}
}

//void read_parameters(ParamInterface *pint, int argc, char **argv)
//{
//no parameters for now
//}


int main(int argc, char **argv)
{
	/* connect to the IPC server, register messages */
	ipc = new IpcStandardInterface;
	if (ipc->Connect(argv[0]) < 0)
		dgc_fatal_error("Could not connect to IPC network.");

	//ParamInterface *pint = new ParamInterface(ipc);
	//read_parameters(pint, argc, argv);

	dgc_TrafficLightState_register_ipc_messages();

	/* setup a shutdown handler */
	signal(SIGINT, shutdown_module);

	//GTK GUI:
	pthread_t gtkGUIThread;
	param.argc = argc;
	param.argv = argv;
	//start the gui thread
	pthread_create(&gtkGUIThread, NULL, run_GUI_function, &param);

	ipc->Dispatch();

	return 0;

}

