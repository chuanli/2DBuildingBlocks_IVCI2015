/****************************************************************************
**
** Copyright (C) 2012 Nokia Corporation and/or its subsidiary(-ies).
** All rights reserved.
** Contact: Nokia Corporation (qt-info@nokia.com)
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** You may use this file under the terms of the BSD license as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of Nokia Corporation and its Subsidiary(-ies) nor
**     the names of its contributors may be used to endorse or promote
**     products derived from this software without specific prior written
**     permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
** $QT_END_LICENSE$
**
****************************************************************************/

#ifndef IMAGEVIEWER_H
#define IMAGEVIEWER_H

#include <QMainWindow>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QPrinter>
#include <QString>
#include "Rep.h"
#include "Para.h"
#include "SenderObject.h"
#include "sideviewer.h"
#include "scribblearea.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
using namespace cv;

QT_BEGIN_NAMESPACE
class QAction;
class QLabel;
class QMenu;
class QScrollArea;
class QScrollBar;
QT_END_NAMESPACE

	QImage Mat2QImage(const cv::Mat3b&src);     
QImage Mat2QImage(const cv::Mat1b&src);    
void imshow(QLabel* const label,  const QImage  &qimg);

//! [0]
class ImageViewer : public QMainWindow
{
	Q_OBJECT

public:
	ImageViewer();

	// function to initialize repetitive building blocks
	void IniImg(); 
	void IniRep(); 
	void IniCooC();
	void IniCooCCross();
	void IniCooCBB();

	//void RepRec();
	//void RepRec2();
	//void RepRecY();

	Rep* rep;
	int synmode;
	int renderRepLabelflag;
	double scaleFactor;
	Mat1d M_conflict;
	Mat1d M_vx;
	Mat1d M_vy;

	QGraphicsScene* scene;
	QGraphicsView* view;
	QGraphicsPixmapItem* imgDisp;
	ScribbleArea* myscribbleArea;
	QGraphicsScene* newscene;
	QGraphicsView* newview;
	sideviewer* newviewer;

	vector<vector<BBItem*>> bbitem;
	vector<vector<BBItem*>> bbitemPos;
	vector<vector<BBItem*>> bbitemNeg;
	vector<vector<BBItem*>> bbitemInput;
	vector<Point2i> rec_bb_list2bb;

	int i_iter_em;
	int ini_label;

	private slots:
		void open();
		void print();
		void save();
		//   void zoomIn();
		//   void zoomOut();
		//   void normalSize();
		//   void fitToWindow();
		//   void about();
		//void synthesis();
		//void synthesisY();

		void synExpandX();
		void synShrinkX();
		void synExpandY();
		void synShrinkY();
		void synFree(); // synthesis without horizontal nor vertical constraint

		void synExpandSampleX();
		void synShrinkSampleX();
		void synExpandSampleY();
		void synShrinkSampleY();
		void synSample(); // synthesis by sampling from cooc statistics
		void synSampleMS();
		void synSampleMSL();
		// rendering related
		void renderRepLabel();
		void renderRepBB();
		void renderInput();

		// EM optimization related
		void synOpt(); // EM optimization using co-occurrence statistics
		void estimationSW(); // sliding window reconstruction of repetitive bb
		void estimationConfig(); // reconfigure bb using co-occurrence statistics
		void estimationCooC(); // estimate structure using co-courrence statistics
		void estimationElastic(); // estimate structure using elasitc model 
		void setGlobalConstraint();
		void maximizationSyn(); // maximize the quality of the synthesized image using estimated bb

		// Constellation related
		void computeKNNbb();
		void computeConsMt();
		void computeConsMtHungarian();
		void computeConsMDS();

		// User editing related
		void updateGuide();
		void scribble();
		void showInput();
		void changetypeBB();
		void synScribble();

		//void renderGuide();
		//void renderBB();
		void synSwitchMode1();
		void synSwitchMSL();
		//void synSwitchMode2();
		//void synSwitchMode3();
		//void synSwitchMode4();
		//void IncrCostSmooth();
		//void DecrCostSmooth();
		//void renderGuideX();
		//void renderGuideY();
		//void updateGuide();
		//void updateGuideX();
		//void updateGuideX_Floating();
		//void updateGuideY();
		//void updateGuideFake();
		//void multiselection();
		//void showInput();
		//void addBB();
		//void changetypeBB();
		//void reconfig();
		//void shiftexpansion();
		//void shiftexpansion2();
		//void autoStitch();
		//void userPaint();
		//void guideStitch();
		//void guideStitch2();
		//void guideStitch3();
		//void scribble();
		//void scribble2();

private:
	void createActions();
	void createActions2();
	void createMenus();
	void createMenus2();

	//QLabel *imageLabel;
	////ScribbleArea
	//ScribbleArea* myscribbleArea;

#ifndef QT_NO_PRINTER
	QPrinter printer;
#endif

	QAction *openAct;
	QAction *saveAct;
	QAction *printAct;
	//   QAction *exitAct;
	//   QAction *saveAct;
	//   QAction *zoomInAct;
	//   QAction *zoomOutAct;
	//   QAction *normalSizeAct;
	//   QAction *fitToWindowAct;
	//   QAction *aboutAct;
	//   QAction *aboutQtAct;

	QAction *synExpandXAct;
	QAction *synShrinkXAct;
	QAction *synExpandYAct;
	QAction *synShrinkYAct;

	//QAction *renderGuideAct;
	QAction *renderRepLabelAct; // render the synthesized bb label map
	QAction *renderRepBBAct; // render the synthesized bb label map
	QAction *synOptAct; // EM optimization using co-occurrence statistics

	QAction *synSwitchMSLAct;
	QAction *synSwitchModeAct1;
	//QAction *synSwitchModeAct2;
	//QAction *synSwitchModeAct3;
	//QAction *synSwitchModeAct4;
	//QAction *IncrCostSmoothAct;
	//QAction *DecrCostSmoothAct;
	//QAction *updateGuideAct;
	//QAction *multiSelectionAct;
	//QAction *showInputAct;
	//QAction *reconfigAct;
	//QAction *autoStitchAct;
	//QAction *userPaintAct;
	QAction *guideStitchAct;
	QAction *scribbleAct;

	QMenu *fileMenu;
	//   QMenu *viewMenu;
	QMenu *helpMenu;
	//QMenu *helpMenuScribble;

protected:

};
//! [0]

#endif
