/* network2D.cpp */

#include "network2D.h"
#include "draw.h"

///////////////////////////////////
////// TODO REFACTOR THAT /////////
///////////////////////////////////

network2D::network2D(int Na, axes *a, const GLubyte c[4])
{
    int i;
    N = Na;

    special_node = 0;
	for (i = 0; i < 4; i++) color[i] = c[i];

    /* allocate memory */
    /* Speicher reservieren für Zeilen-Zeiger */
	X = (float **) malloc(N * sizeof(float *));
    if (NULL == X) {
		fprintf(stderr, "Fehler: Zu wenig Speicher.\n");
		exit(-1);
	}

	/* Speicher reservieren für Spalten */
	for (i = 0; i < N; i++) {
		X[i] = (float *) calloc(2, sizeof(float));
		if (NULL == X[i]) {
			fprintf(stderr, "Fehler: Zu wenig Speicher.\n");
			exit(-1);
		}
	}


	size = (float*) calloc(N, sizeof(float));

    for (i = 0; i < N; i++) {
        X[i][0] = 0.0f;
        X[i][1] = 0.0f;
		size[i] = 0.0f;
    }
    pointer = 0;

    /* link to axis */
    px = a->px;
    py = a->py;
    A = a;
    width = 0.5*a->width;
    height = 0.5*a->height;


	/* Speicher reservieren für Zeilen-Zeiger */
	edges = (unsigned char **) malloc(N * sizeof(unsigned char *));
    if (NULL == edges) {
		fprintf(stderr, "Fehler: Zu wenig Speicher.\n");
		exit(-1);
	}

	/* Speicher reservieren für Spalten */
	for (i = 0; i < N; i++) {
		edges[i] = (unsigned char *) calloc(N, sizeof(unsigned char));
		if (NULL == edges[i]) {
			fprintf(stderr, "Fehler: Zu wenig Speicher.\n");
			exit(-1);
		}
	}

}

void network2D::draw()
{
    /* draw nodes */

	int i,j;
	glColor4ub(255,255,255,255);

	for (i = 0; i < N; i++) {
		if (size[i] > 0) { //node exists
			draw_fill_square(X[i][0], X[i][1], 0.005);
		}
	}

	glColor4ub(255,255,255,96);

	for (i = 0; i < N; i++) {
		if (size[i] > 0) { //node exists
			draw_fill_square(X[i][0], X[i][1], 0.005+0.01*size[i]);
		}
	}


	/*glColor4ub(255,192,0,128);
	if (size[special_node] > 0)
		draw_fill_rect(X[special_node][0], X[special_node][1], 0.02);
	*/
	glColor4ub(0,128,255,128);
	if (size[activated_node] > 0)
		draw_fill_square(X[activated_node][0], X[activated_node][1], 0.015);

	/* draw edges */
	glLineWidth(1.0f);

	for (i = 0; i < N; i++)
		for (j = 0; j < N; j++)
			if (i != j)
				if (edges[j][i])
				{
					glColor4ub(255,255,255,edges[j][i]);
					draw_line(X[i], X[j]);
				}

}

void network2D::special(int n)
{

	if (n >= 0 && n < N)
		special_node = n;
	else {
    	fprintf(stderr, "Fehler: Falscher Knotenindex");
    	exit(-1);
    }
}

void network2D::activated(int n)
{

	if (n >= 0 && n < N)
		activated_node = n;
	else {
    	fprintf(stderr, "Fehler: Falscher Knotenindex");
    	exit(-1);
    }
}

void network2D::update_node(int n, float x0, float x1, float s)
{
    if (n >= 0 && n < N) {
   		X[n][0] = px + x0*width;
   		X[n][1] = py + x1*height;
   		size[n] = s;
    } else {
    	fprintf(stderr, "Fehler: Falscher Knotenindex");
    	exit(-1);
    }
}

void network2D::update_edge(int i, int j, unsigned char op)
{
    if (j>=0 && j<N && i>=0 && i<N) {
    	edges[i][j] = op;
 	} else {
    	fprintf(stderr, "Fehler: Falscher Kantenindex");
    	exit(-1);
    }
}

/* network2D.cpp */

