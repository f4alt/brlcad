/*                      R I P P L E _ A N I M . C P P
 * BRL-CAD
 *
 * Copyright (c) 2004-2022 United States Government as represented by
 * the U.S. Army Research Laboratory.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * version 2.1 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this file; see the file named COPYING for more
 * information.
 */
/** @file proc-db/ripple_anim.cpp
 *
 * Program to create a directory of gif frames for a ripple animation
 * utilizing libwdb
 * USE: run the executable ./ripple_anim
 * by default, it will create a directory of a .g file, run script, and
 * empty frames directory. Each step for the animation is a combination
 * in the .g in the form of all.tx where x is the frame number.
 * The executable run.sh will iterate over all steps and rt to save
 * each frame as a .png to ra/frames. This directory can then be
 * gif-ified to see the animation
 *
 */

#include "common.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fstream>

#include "vmath.h"
#include "bu/app.h"
#include "bu/getopt.h"
#include "bn.h"
#include "raytrace.h"
#include "rt/geom.h"
#include "wdb.h"
#include "string.h"

#define DEFAULT_DIRECTORY "ra"
#define DEFAULT_FILENAME_BASE "ripple_anim.g"
#define DEFAULT_FRAMES 8
#define DEFAULT_ORIGIN 0.0
#define DEFAULT_DIMENSION 1.0
#define DEFAULT_GRID_SIZE 10
#define DEFAULT_RANDOM_COLORS true
#define DEFAULT_FLOOR_COLOR "76 71 87"
#define DEFAULT_OFFSET 4

#define FLOOR_ID 0
#define SCENE_ID_START 1

#define MAX_GRID_SIZE 20
#define MAX_INPUT_LENGTH 48

struct rt_wdb *db_fp;
struct wmember *wm_arr;
struct stat info;
int looplen;

struct params {
    std::string path;
    point_t origin, dimensions;
    int maxLevel;
    int offset;
    int gridSize;
    unsigned char rgb[2 * MAX_GRID_SIZE -1][3];
};
typedef struct params params_t;

void usage (char *n) {
  fprintf(stderr,
	"\nUsage: %s [options]\n\
  -o fileName     -- specify output file (default %s)\n\
  -f #            -- specify amount of frames for ONE cubes full motion (default %d)\n\
  -d dirName      -- specify output directory (default %s/)\n\
  -g #            -- specify grid size (default %d)\n\
  -n #            -- specify animation cascade start offset in form 1/x (default 1/%d)\n\
  -c              -- toggles random color generation (default %s)\n\n",
  n,
  DEFAULT_FILENAME_BASE,
  DEFAULT_FRAMES,
  DEFAULT_DIRECTORY,
  DEFAULT_GRID_SIZE,
  DEFAULT_OFFSET,
  DEFAULT_RANDOM_COLORS == true ? "on" : "off"
  );
}

// ensure output directory is already / can be created
bool check_directory(const char* dir) {
  if (stat( dir, &info ) != 0 ) {
    printf( "creating %s/\n", dir );

    if (!mkdir(dir, 0777)) {
      char internal[MAX_INPUT_LENGTH+7];
      sprintf(internal, "%s/frames", dir);

      // if create is good, create internal /frames
      if (!mkdir(internal, 0777)) {
        return true;
      }
      return false;
    }
    return false;
  } else if (info.st_mode & S_IFDIR ) {
    return true;
  }

  return false;
}

// initializes all values in params struct and sets global 'looplen' variable
void initializeInfo(params_t *p,
                    std::string _path,
                    point_t _p1,
                    int frames,
                    int _offset,
                    point_t _dimensions,
                    int _gridSize,
                    bool rand_colors) {
  p->path = _path;
  VMOVE(p->origin, _p1);
  p->maxLevel = frames/2;
  p->offset = _offset;
  VMOVE(p->dimensions, _dimensions);
  p->gridSize = _gridSize;

  // set all rgb's to white by default
  memset(p->rgb, 255, sizeof(p->rgb));
  // if user wants random cube colors, generate rgb values
  if (rand_colors) {
    srand (time(NULL));
    for (int i=0; i < 2 * _gridSize -1; i++) {
      VSET(p->rgb[i], rand() % 255, rand() % 255, rand() % 255);
    }
  }


  // since we want subsequent rows to start when the previous row is half way
  // on the ascent, we set the offset to be half the total height (frames/2)
  // divided by the user specified height 'offset'
  // for the total loop, we need all the rows but one times the offset + the
  // amount of frames for the last rows total motion
  looplen = ((2 * _gridSize - 2) * (frames / 2) / _offset) + frames - 1;
}

void createFloor(params_t *params) {
  unsigned char rgb[3];
  sscanf(DEFAULT_FLOOR_COLOR, "%hhu %hhu %hhu", &rgb[0], &rgb[1], &rgb[2]);

  // make rpp for ground that spans all cubes +1
  point_t g1, g2;
  VSET(g1, params->origin[X]-1, params->origin[Y]-1, -0.1);
  VSET(g2,
       params->gridSize * params->dimensions[X] * params->dimensions[X] + 1,
       params->gridSize * params->dimensions[Y] * params->dimensions[Y] + 1,
       0);

  mk_rpp(db_fp, "floor.s", g1, g2);

  (void)mk_addmember("floor.s", &wm_arr[FLOOR_ID].l, NULL, WMOP_UNION);

  int is_region = 1;
  mk_lcomb(db_fp,
     "floor.r",
     &wm_arr[FLOOR_ID],
     is_region,
     "plastic",
     "di=.8 sp=.2",
     rgb,
     0);
}

void createScene(params_t* params, int frame_no) {
  std::string step = "all.t";
  step.append(std::to_string(frame_no));

  // add all cubes and floor to scene
  for (int row=0; row < params->gridSize; row++) {
    for (int col=0; col < params->gridSize; col++) {
      // create unique shape name
      std::string currRItem = "box";
      currRItem.append(std::to_string(row));
      currRItem.append("_");
      currRItem.append(std::to_string(col));
      currRItem.append(".r.");
      currRItem.append(std::to_string(frame_no));

      int is_region = 1;
      int startOffset = looplen + params->gridSize * params->gridSize * frame_no;
      mk_lcomb(db_fp,
        currRItem.c_str(),
        &wm_arr[startOffset + params->gridSize * row + col],
        is_region,
        "plastic",
        "di=.8 sp=.2",
        params->rgb[row + col],
        0);

      mk_addmember(currRItem.c_str(), &wm_arr[SCENE_ID_START + frame_no].l, NULL, WMOP_UNION);
    }
  }

  mk_addmember("floor.r", &wm_arr[SCENE_ID_START + frame_no].l, NULL, WMOP_UNION);
  mk_lfcomb(db_fp, step.c_str(), &wm_arr[SCENE_ID_START + frame_no], 0);
}

bool createFrame(params_t *params, int frame_no) {
  // make grid of cubes
  for (int row=0; row < params->gridSize; row++) {
    for (int col=0; col < params->gridSize; col++) {
      // calculate x and y translation per shape
      point_t newp1;
      VMOVE(newp1, params->origin);
      newp1[X] += row * params->dimensions[X];
      newp1[Y] += col * params->dimensions[Y];
      point_t newp2;
      VMOVE(newp2, params->dimensions);
      newp2[X] += row * params->dimensions[X];
      newp2[Y] += col * params->dimensions[Y];

      // create unique shape name
      std::string currItem = "box";
      currItem.append(std::to_string(row));
      currItem.append("_");
      currItem.append(std::to_string(col));
      currItem.append(".s.");
      currItem.append(std::to_string(frame_no));

      // make rpp
      mk_rpp(db_fp, currItem.c_str(), newp1, newp2);

      // calculate and set height translation
      mat_t trans;
      MAT_IDN(trans); // set identity matrix
      // trans in z direction by proportional to frame
      float height_dif = (params->maxLevel / params->offset);
      if (((row + col) * height_dif) < (frame_no)) {
        float height = (((float)frame_no - (float)(row + col) * height_dif) / (float)params->maxLevel) * params->dimensions[Z];
        if (height > 1.0f) {
          height = std::max(2.0f - height, 0.0f);
        }
        trans[11] = height;
      }

      // add members
      int startOffset = looplen + params->gridSize * params->gridSize * frame_no;
      (void)mk_addmember(currItem.c_str(),
      &wm_arr[startOffset + params->gridSize * row + col].l,
      trans,
      WMOP_UNION);
    }
  }

  // create group with all cubes and floor
  createFloor(params);  // technically redundant after initial creation
  createScene(params, frame_no);

  return true;
}

// creates rt script that will traverse all frames and create
// cooresponding .png with animation in a45 e145 view
void creatertscript(std::string dir, std::string fileName, int gridSize) {
  char cmd[1800];

  // TODO:
  // viewize only accounts for square grid
  // eye_pt calculation is not quite exact
  sprintf(cmd, "#!/bin/bash\n\n\
echo NOTE: this script uses rt, ensure it is in your path\n\n\
## Create forward motion\n\
for (( frame=0; frame<=%d; frame++ ))\n\
  do\n\
  rt -M \\\n\
   -o frames/ra_frame_$frame.png\\\n\
   $*\\\n\
   '%s'\\\n\
   'all.t'$frame'' \\\n\
   2>> run.rt.log\\\n\
   <<EOF\n\
   viewsize %d;\n\
   orientation 1.46446609406726e-01 3.53553390593274e-01 8.53553390593274e-01 3.53553390593274e-01;\n\
   eye_pt %f %f %f;\n\
  start 0; clean;\n\
  end;\n\
EOF\n\
done\n\n\
echo done: frames saved in /frames\n",
  looplen,
  fileName.c_str(),
  (gridSize + 2) * 2,
  (gridSize + 2) * 2.0,
  (gridSize + 2) * 2.0,
  (gridSize + 2) * 2.0 + 2.0
);

  // create output executable called run.sh
  char foutfile [MAX_INPUT_LENGTH + 6];
  sprintf(foutfile, "%s/run.sh", dir.c_str());
  std::ofstream outfile (foutfile);
  outfile << cmd;

  outfile.close();

  // make file executable
  std::string exec = "chmod +x ";
  exec.append(foutfile);
  if (!system(exec.c_str())) {
    printf("executable [run.sh] created in %s/\n", dir.c_str());
  } else {
    printf("ERROR creating executable\n");
  }
}


int main(int argc, char **argv) {
  params_t params;
  point_t origin, dimensions;
  int optc;

  // init
  std::string fileName = DEFAULT_FILENAME_BASE;
  std::string dir = DEFAULT_DIRECTORY;
  int frames = DEFAULT_FRAMES;
  int gridSize = DEFAULT_GRID_SIZE;
  VSETALL(dimensions, DEFAULT_DIMENSION);
  VSETALL(origin, DEFAULT_ORIGIN);
  bool rand_colors = DEFAULT_RANDOM_COLORS;
  int offset = DEFAULT_OFFSET;
  bu_setprogname(argv[0]);

  // get users command line options to override specified defaults
  while ((optc = bu_getopt(argc, argv, "o:f:d:g:cn:?")) != -1) {
    switch (optc) {
        case 'd':  /* Use a user-defined directory */
          dir = bu_optarg;
          break;
        case 'o':  /* Use a user-defined filename */
          fileName = bu_optarg;
          break;
        case 'f': /* Use a user-defined amount of frames */
          frames = atoi(bu_optarg);
          break;
        case 'g': /* Use a user-defined grid size */
          gridSize = atoi(bu_optarg);
          if (gridSize > MAX_GRID_SIZE) {
            printf("ERROR max grid size %d\n", MAX_GRID_SIZE);
            gridSize = MAX_GRID_SIZE;
          }
          break;
        case 'c': /* toggles random colors option */
          rand_colors = !rand_colors;
          break;
        case 'n': /* user specified cascade start offset */
          offset = atoi(bu_optarg);
          break;
        default:
          usage(argv[0]);
          bu_exit(0, NULL);
    }
  }

  // check for / create output directory
  if (!check_directory(dir.c_str())) {
    bu_exit(1, "ERROR: %s directory could not be created", dir.c_str());
  }

  // verify users inputs with console print
  bu_log("using grid size [%d]\nnumber of frames [%d]\nstart offset[1/%d]\n", gridSize, frames, offset);

  // add file name to dir for file creation
  std::string path = dir + "/" + fileName;
  initializeInfo(&params, path, origin, frames, offset, dimensions, gridSize, rand_colors);

  // open current db
  if ((db_fp = wdb_fopen(params.path.c_str())) == NULL) {
    perror(params.path.c_str());
    return false;
  }
  // set basic title and units
  mk_id_units(db_fp, params.path.c_str(), "mm");

  // initialize wm array for the 1 floor + x scenes + x * y cubes in grid
  int num_members = 1 + (looplen+1) + (looplen+1) * (gridSize*gridSize);
  wm_arr = (struct wmember *)bu_malloc(sizeof(struct wmember)* num_members, "alloc wm_arr");
  for (int i = 0; i < num_members; i++) {
    BU_LIST_INIT(&(wm_arr[i].l));
  }

  // main driver to create one region per frame
  for (int i=0; i <= looplen; i++) {
    createFrame(&params, i);
  }

  // create run.sh and put in dir
  creatertscript(dir, fileName, gridSize);

  // close and cleanup
  wdb_close(db_fp);
  bu_free(wm_arr, "free wm_arr");
  bu_log("done.\n");

  return 0;
}


/*
* Local Variables:
* mode: C
* tab-width: 8
* indent-tabs-mode: t
* c-file-style: "stroustrup"
* End:
* ex: shiftwidth=4 tabstop=8
*/
