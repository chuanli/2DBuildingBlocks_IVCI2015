#ifndef REP_H
#define REP_H
#include <vector>
#include <QLabel>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QKeyEvent>
#include <QApplication>
#include <QGraphicsObject>
#include "Para.h"
#include "SenderObject.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include "../src/gui/graphicsview/qgraphicsitem.h"

using namespace cv;
using namespace std;

class BBItem : public QGraphicsPolygonItem, public QObject
{
public:
	BBItem(int a, int b);

	int x_start;
	int y_start;
	int idx_s; // index to rec_shift_list
	int idx_item; // index to rec_bb_list
	bool flag_changed;

	int x_shift;
	int y_shift;
	int x_shift_ori;
	int y_shift_ori;
	int bb_type;
	int bb_idx;

	int x_fullres;
	int y_fullres;
	int w_fullres; 
	int h_fullres;
	int x_scaled;
	int y_scaled;
	int w_scaled; 
	int h_scaled;

	// idx to rec_bb_list
	Point2i idx2rec_bb_list;

signals:

protected:
	void mousePressEvent(QGraphicsSceneMouseEvent *event);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
	void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
	void updateItem(QGraphicsSceneMouseEvent *event);


	//void updateX(QGraphicsSceneMouseEvent *event);
	//void updateY(QGraphicsSceneMouseEvent *event);

private:

};


class Rep
{
public:
	Rep(void);
	~Rep(void);

	// Input image
	QImage* qimgInput_fullres;
	QImage* qimgInput_scaled;
	QImage* qimgInputGray_fullres;
	QImage* qimgInputGray_scaled;
	QLabel* qlabelInput_fullres;
	QLabel* qlabelInput_scaled;
	QLabel* qlabelInputGray_fullres;
	QLabel* qlabelInputGray_scaled;
	Mat3b imgInput_fullres;
	Mat3b imgInput_scaled;
	Mat1b imgInputGray_fullres;
	Mat1b imgInputGray_scaled;

	// Input label
	QImage* qimgInputlabel_fullres;
	QImage* qimgInputlabel_scaled;
	Mat1b imgInputlabel_fullres;
	Mat1b imgInputlabel_scaled;

	// Input internal label
	QImage* qimgInputlabelinterX_fullres; 
	QImage* qimgInputlabelinterX_scaled;
	Mat1d imgInputlabelinterX_fullres;
	Mat1d imgInputlabelinterX_scaled;
	QImage* qimgInputlabelinterY_fullres;
	QImage* qimgInputlabelinterY_scaled;
	Mat1d imgInputlabelinterY_fullres;
	Mat1d imgInputlabelinterY_scaled;

	// Synthesized image
	QImage* qimgSyn_fullres;
	QImage* qimgSyn_scaled;
	QImage* qimgSynGray_fullres;
	QImage* qimgSynGray_scaled;
	QLabel* qlabelSyn_fullres;
	QLabel* qlabelSyn_scaled;
	QLabel* qlabelSynGray_fullres;
	QLabel* qlabelSynGray_scaled;
	Mat3b imgSyn_fullres;
	Mat3b imgSyn_scaled;
	Mat1b imgSynGray_fullres;
	Mat1b imgSynGray_scaled;
	QImage* qimgSynlabelColor_fullres;
	QLabel* qlabelSynlabelColor_fullres;
	Mat1b gcolabelSyn_fullres;
	Mat1b gcolabelSyn_scaled;

	// user guidance
	QImage* qimgUserGuide_fullres;
	QImage* qimgUserGuide_scaled;
	QImage* qimgUserGuideGray_fullres;
	QImage* qimgUserGuideGray_scaled;
	Mat3b imgUserGuide_fullres;
	Mat3b imgUserGuide_scaled;
	Mat1b imgUserGuideGray_fullres;
	Mat1b imgUserGuideGray_scaled;
	//data related to guidance map -- not sure what is this thing ... 
	Mat1b imgGuide_fullres;
	Mat1b imgGuide_scaled;

	// dimension
	int rowsInput_scaled;
	int colsInput_scaled;
	int rowsSyn_scaled;
	int colsSyn_scaled;
	int rowsPaint_scaled;
	int colsPaint_scaled;
	int numPixelInput_scaled;
	int numPixelSyn_scaled;

	int rowsInput_fullres;
	int colsInput_fullres;
	int rowsSyn_fullres;
	int colsSyn_fullres;
	int rowsPaint_fullres;
	int colsPaint_fullres;
	int numPixelInput_fullres;
	int numPixelSyn_fullres;

	// shifts
	int num_shiftX;
	int dist_shiftX_scaled;
	vector<int> list_shiftX_scaled;
	int dist_shiftX_fullres;
	vector<int> list_shiftX_fullres;
	int num_shiftY;
	int dist_shiftY_scaled;
	vector<int> list_shiftY_scaled;
	int dist_shiftY_fullres;
	vector<int> list_shiftY_fullres;
	int num_shiftXY;
	vector<Point2i> list_shiftXY_scaled;
	vector<Point2i> list_shiftXY_fullres;

	// shift for MSL
	int num_shiftMSL;
	vector<Point2i> list_shiftXY_MSL_scaled;
	vector<Point2i> list_shiftXY_MSL_fullres;


	// Repetitive Building Blocks data ... 
	int numRep; 
	vector<int>sizeRep;
	vector<vector<int>> repX_scaled; // repetitions X scaled
	vector<vector<int>> repY_scaled; // repetitions Y scaled
	vector<vector<int>> repW_scaled; // repetitions W scaled
	vector<vector<int>> repH_scaled; // repetitions H scaled
	vector<vector<int>> repX_fullres; // repetitions X fullres
	vector<vector<int>> repY_fullres; // repetitions Y fullres
	vector<vector<int>> repW_fullres; // repetitions W fullres
	vector<vector<int>> repH_fullres; // repetitions H fullres
	int numBB;

	// co-occurrence statistics
	int numCooC; 
	vector<int>sizeCooC;
	vector<vector<int>> coocX_scaled;
	vector<vector<int>> coocY_scaled;
	vector<vector<int>> coocX_fullres;
	vector<vector<int>> coocY_fullres;

	// co-occurrent statistics of BB
	int num_bbstatisticsX;
	int num_bbstatisticsY;
	vector<int> list_bbstatisticsX;
	vector<int> list_bbstatisticsY;
	int expansion_bbstatisticsX;
	int expansion_bbstatisticsY;
	int expansion_bbstatisticsX_scaled;
	int expansion_bbstatisticsY_scaled;

	// votes
	Mat1d imgVote_scaled;
	vector<int>sizeVote;
	vector<vector<int>> voteX_scaled;
	vector<vector<int>> voteY_scaled;
	vector<vector<int>> voteX_fullres;
	vector<vector<int>> voteY_fullres;

	// list of naively reconstructed BB, this is the output of the estimation step 
	vector<int> rec_shift_list;
	vector<vector<pair<int, int>>> rec_bb_list;

	vector<int> neg_shift_list;
	vector<vector<pair<int, int>>> neg_bb_list;

	// list of reconfigured BB, this is the (intermediate) output of the maximization step 
	vector<int> config_shift_list;
	vector<vector<pair<int, int>>> config_bb_list;

	// for every pixel, a list of KNN building blocks
	vector<vector<Point3d>> pixel2bb; // type, vx, vy


	// a constellation distance matrix for pairwise pixels
	


};

#endif