#ifndef PARA_H
#define PARA_H

#include <vector>
#include <QString>
#include <QObject>
#include <QDebug>
#include "SenderObject.h"

using namespace std;

extern  QString directoryname;
extern  QString filename_imgInput;
extern  QString filename_repInput;
extern  QString filename_coocInput;
//extern  QString filename_userguideInput;


extern  double scalerRes;
extern  double scalerSynX; // scaling factor for synthesized image, X dimension
extern  double scalerSynY; // scaling factor for synthesized image, Y dimension

extern  int max_num_shift_scaled;

extern  int num_shiftX_scaled;
extern  int num_shiftY_scaled;

extern int cost_data_inf;
extern int cost_smooth_inf;
extern int cost_smooth_label;
extern int cost_data_guide_inf;
extern float cost_smooth_pixel_scaler;
extern float cost_data_pixel_scaler;
extern float cost_smooth_constellation_scaler;
extern float cost_data_prior_scaler;


extern  int max_iter_rec;
extern  int incr_iter_rec;

extern double expansion_stepX;
extern double expansion_stepY;

extern SenderObject* globalsender;

extern bool flag_multiselection;
extern bool flag_sideselection;

extern int sel_bb_type;

extern int sel_bb_idx;

extern int global_rowsSyn_scaled;
extern int global_colsSyn_scaled;
extern int global_rowsInput_scaled;
extern int global_colsInput_scaled;

extern int global_rowsSyn_fullres;
extern int global_colsSyn_fullres;
extern int global_rowsInput_fullres;
extern int global_colsInput_fullres;

extern double global_cover_in_thresh;
extern double global_cover_out_thresh;

extern bool flag_syn;
extern bool flag_scribble;
extern bool flag_interactive;
extern bool flag_MS;
extern bool flag_scribblemap;

extern bool flag_constellation;

extern int step_reconfig;

extern int num_iter_autostitch;

extern double scalerPaintX;
extern double scalerPaintY;

extern int max_iter_em;

extern double ini_cooc_dist_bb;
extern double ini_cooc_dist;
extern double ini_cooc_dist_ratio;

extern double cur_cooc_dist_bb;
extern double cur_cooc_dist;
extern double cur_cooc_dist_ratio;

extern int max_cooc_KNN_num;
extern double max_cooc_KNN_dist;

extern int border;
extern int border_fullres;

extern int ini_num_KNN;
extern int cur_num_KNN;

extern double max_dist_constellation;

extern int globalnumRep;

extern int x_shift_lowbd;
extern int x_shift_upbd;
extern int y_shift_lowbd;
extern int y_shift_upbd;


extern int default_syn_mode;
#endif