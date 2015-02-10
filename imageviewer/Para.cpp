#include "Para.h"

//QString directoryname = "C:/Chuan/data/DemoIVCI2015/2DSynthesis";
//QString filename_imgInput = "C:/Chuan/data/DemoIVCI2015/2DSynthesis/Chuan(1).jpg";
//QString filename_repInput = "C:/Chuan/data/DemoIVCI2015/2DSynthesis/Chuan(1)Detection.txt";
//QString filename_coocInput = "C:/Chuan/data/DemoIVCI2015/2DSynthesis/Chuan(1)BBOccurrenceDetection.txt";

QString directoryname = "";
QString filename_imgInput = "fac(46).jpg";
QString filename_repInput = "fac(46)Detection.txt";
QString filename_coocInput = "fac(46)BBOccurrenceDetection.txt";

//QString filename_imgInput = "C:/Chuan/Dropbox/Project/2DBuildingBlock/SiggraphAsia/Code/ImageSyn/imageviewer/data/fac(20).jpg";
//QString filename_repInput = "C:/Chuan/Dropbox/Project/2DBuildingBlock/SiggraphAsia/Code/ImageSyn/imageviewer/data/fac(20)GT.txt";
//QString filename_coocInput = "C:/Chuan/Dropbox/Project/2DBuildingBlock/SiggraphAsia/Code/ImageSyn/imageviewer/data/fac(20)OccurrenceGT.txt";
//QString filename_userguideInput = "C:/Chuan/Dropbox/Project/2DBuildingBlock/SiggraphAsia/Code/ImageSyn/imageviewer/data/fac(20)Guide10.png";

//QString filename_imgInput = "C:/Chuan/Dropbox/Project/2DBuildingBlock/SiggraphAsia/Code/ImageSyn/imageviewer/data/fac(20).jpg";
//QString filename_repInput = "C:/Chuan/Dropbox/Project/2DBuildingBlock/SiggraphAsia/Code/ImageSyn/imageviewer/data/fac(20)Detection.txt";
//QString filename_coocInput = "C:/Chuan/Dropbox/Project/2DBuildingBlock/SiggraphAsia/Code/ImageSyn/imageviewer/data/fac(20)OccurrenceDetection.txt";
//QString filename_userguideInput = "C:/Chuan/Dropbox/Project/2DBuildingBlock/SiggraphAsia/Code/ImageSyn/imageviewer/data/fac(20)Guide10.png";


//QString filename_imgInput = "C:/Chuan/Dropbox/Project/2DBuildingBlock/SiggraphAsia/Code/ImageSyn/imageviewer/data/fac(7).jpg";
//QString filename_repInput = "C:/Chuan/Dropbox/Project/2DBuildingBlock/SiggraphAsia/Code/ImageSyn/imageviewer/data/fac(7)GT.txt";
//QString filename_coocInput = "C:/Chuan/Dropbox/Project/2DBuildingBlock/SiggraphAsia/Code/ImageSyn/imageviewer/data/fac(7)OccurrenceCrossGT.txt";
//QString filename_userguideInput = "C:/Chuan/Dropbox/Project/2DBuildingBlock/SiggraphAsia/Code/ImageSyn/imageviewer/data/fac(7)Guide10.png";

//QString directoryname = "C:/Chuan/Local/Data/Syn/Input";
//QString filename_imgInput = "C:/Chuan/Local/Data/Syn/Input/fac(7).jpg";
//QString filename_repInput = "C:/Chuan/Local/Data/Syn/Input/fac(7)Detection.txt";
////QString filename_repInput = "C:/Chuan/Local/Data/Syn/Input/fac(7)GT.txt";
//QString filename_coocInput = "C:/Chuan/Local/Data/Syn/Input/fac(7)BBOccurrenceDetection.txt";
////QString filename_coocInput = "C:/Chuan/Local/Data/Syn/Input/fac(7)BBOccurrenceGT.txt";

//QString filename_userguideInput = "C:/Chuan/Dropbox/Project/2DBuildingBlock/SiggraphAsia/Code/ImageSyn/imageviewer/dataChuan/Chuan(1)Guide10.png";

//QString filename_imgInput = "C:/Chuan/Dropbox/Project/2DBuildingBlock/SiggraphAsia/Code/ImageSyn/imageviewer/dataChuan/img(0).jpg";
//QString filename_repInput = "C:/Chuan/Dropbox/Project/2DBuildingBlock/SiggraphAsia/Code/ImageSyn/imageviewer/dataChuan/img(0)GT.txt";
//QString filename_coocInput = "C:/Chuan/Dropbox/Project/2DBuildingBlock/SiggraphAsia/Code/ImageSyn/imageviewer/dataChuan/img(0)OccurrenceCrossGT.txt";
//QString filename_userguideInput = "C:/Chuan/Dropbox/Project/2DBuildingBlock/SiggraphAsia/Code/ImageSyn/imageviewer/dataChuan/img(0)Guide10.png";

double scalerRes = 0.125;
double scalerSynX = 1; // scaling factor for synthesized image, X dimension
double scalerSynY = 1; // scaling factor for synthesized image, Y dimension

int num_shiftX_scaled = 1;
int num_shiftY_scaled = 1;
int incr_iter_rec = 1; // do not change for synthesis using statistical sampling

//double expansion_stepX = 0.2; // scaling step
//double expansion_stepY = 0.275; // scaling step

//double expansion_stepX = 0.47; // scaling step
//double expansion_stepY = 0.13; // scaling step

double expansion_stepX = 0.16; // scaling step
double expansion_stepY = 0.2; // scaling step

//double expansion_stepX = 0.2; // scaling step
//double expansion_stepY = 0.2; // scaling step

int cost_data_inf = 1000000;
int cost_smooth_inf = 1000000;
float cost_smooth_pixel_scaler = 1;
float cost_smooth_constellation_scaler = 10;
float cost_data_pixel_scaler = 1;
float cost_data_prior_scaler = 10;
int cost_smooth_label = 50;
int cost_data_guide_inf = 1000000;

int max_iter_rec = 10;
bool flag_multiselection = false;
bool flag_sideselection = false;
SenderObject* globalsender = new SenderObject;

int sel_bb_type = -1;
int sel_bb_idx = -1;
int global_rowsSyn_scaled = 0;
int global_colsSyn_scaled = 0;
int global_rowsInput_scaled = 0;
int global_colsInput_scaled = 0;
int global_rowsSyn_fullres = 0;
int global_colsSyn_fullres = 0;
int global_rowsInput_fullres = 0;
int global_colsInput_fullres = 0;

bool flag_syn = false;
double global_cover_in_thresh = 0.5;
double global_cover_out_thresh = 0.1;
int step_reconfig = 8;
int num_iter_autostitch = 2;
double scalerPaintX = 1.5;
double scalerPaintY = 1.0;

bool flag_scribble = false;
bool flag_interactive = false;
bool flag_constellation = false;
bool flag_MS = false;

int max_iter_em = 1;

double ini_cooc_dist = 1;
double ini_cooc_dist_bb = 15;
double ini_cooc_dist_ratio = 0.25;
double cur_cooc_dist = ini_cooc_dist;
double cur_cooc_dist_bb = ini_cooc_dist_bb;
double cur_cooc_dist_ratio = ini_cooc_dist_ratio;

int max_cooc_KNN_num = 5;
double max_cooc_KNN_dist = 2;

//int border = 6/(0.125/scalerRes);
int border = 6/(0.125/scalerRes);
int border_fullres = 32; // a fixed border constraint 


int ini_num_KNN = 5; // KNN for constellation model
int cur_num_KNN = ini_num_KNN;

double max_dist_constellation = 0;
int globalnumRep = 0;

int x_shift_lowbd = -10;
int x_shift_upbd = 10;
int y_shift_lowbd = -10;
int y_shift_upbd = 10;

int default_syn_mode = 1;