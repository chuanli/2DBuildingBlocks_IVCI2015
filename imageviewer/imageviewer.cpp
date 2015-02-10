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
#include <QtGui>
#include <cmath>
#include "imageviewer.h"
#include "sideviewer.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <QPen>
#include "gco\GCoptimization.h"
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include "hungarian/hungarian.h"
#include "simplematrix/SimpleMatrix.h"

using namespace std;

vector<Point2i> gcLabels;
vector<Point2i> gcNodes;
vector<Point2i> gcNodesSyn;
vector<Point2i> gcNodesInput;

vector<Point2i> gcLabels_fullres;
vector<Point2i> gcNodes_fullres;
vector<Point2i> gcNodesSyn_fullres;
vector<Point2i> gcNodesInput_fullres;

int inputcols;
int inputrows;
int syncols;
int synrows;
int inputcols_fullres;
int inputrows_fullres;
int syncols_fullres;
int synrows_fullres;

Mat1b gcImage;
Mat1b gcImage_fullres;

Mat1b gcLabel;
Mat1d gcLabelinterX;
Mat1d gcLabelinterY;

Mat1b gcLabel_fullres;
Mat1d gcLabelinterX_fullres;
Mat1d gcLabelinterY_fullres;

Mat1d gcGuide; // guidance map of user editing, the label is for shifts
Mat1d gcGuideMask;
Mat1b gcGuideScribble; // guidance map of user scribble, the label is for bb type
Mat1b gcGuideScribbleMask;

Mat1b gcGuide_MSL_fullres; // upsampled label map for correcting mistakes at the finer resolution

Mat1d gcPosGuide;
Mat1d gcNegGuide;
Mat1d M_constellation;

vector<vector<int>> list_boundary_constraint;
vector<vector<int>> regioninvalid_boundary_constraint;

vector<vector<double>> colorList;
QPen pen;
inline vector<double> makeVector3f(float x, float y, float z) {
	vector<double> v;
	v.resize(3);
	v[0] = x; v[1] = y; v[2] = z;
	return v;
}

static void meshgrid(const cv::Mat &xgv, const cv::Mat &ygv,
	cv::Mat1d &X, cv::Mat1d &Y)
{
	cv::repeat(xgv.reshape(1,1), ygv.total(), 1, X);
	cv::repeat(ygv.reshape(1,1).t(), 1, xgv.total(), Y);
}

// helper function (maybe that goes somehow easier)
static void meshgridTest(const cv::Range &xgv, const cv::Range &ygv,
	cv::Mat1d &X, cv::Mat1d &Y)
{
	std::vector<double> t_x, t_y;
	for (int i = xgv.start; i <= xgv.end; i++) t_x.push_back((double)i);
	for (int i = ygv.start; i <= ygv.end; i++) t_y.push_back((double)i);
	meshgrid(cv::Mat(t_x), cv::Mat(t_y), X, Y);
}

inline double round( double d )
{
	return floor( d + 0.5 );
}

int min(int x, int y)
{
	return (x < y) ? x : y;
}

int max(int x, int y)
{
	return (x > y) ? x : y;
}

int** array_to_matrix(int* m, int rows, int cols) {
	int i,j;
	int** r;
	r = (int**)calloc(rows,sizeof(int*));
	for(i=0;i<rows;i++)
	{
		r[i] = (int*)calloc(cols,sizeof(int));
		for(j=0;j<cols;j++)
			r[i][j] = m[i*cols+j];
	}
	return r;
}

QImage Mat2QImage(const cv::Mat3b &src) {
	QImage dest(src.cols, src.rows, QImage::Format_ARGB32);
	for (int y = 0; y < src.rows; ++y) {
		const cv::Vec3b *srcrow = src[y];
		QRgb *destrow = (QRgb*)dest.scanLine(y);
		for (int x = 0; x < src.cols; ++x) {

			destrow[x] = qRgba(srcrow[x][2], srcrow[x][1], srcrow[x][0], 255);
		}
	}
	return dest;
}

QImage Mat2QImage(const cv::Mat1b &src) {
	QImage dest(src.cols, src.rows, QImage::Format_ARGB32);
	for (int y = 0; y < src.rows; ++y) {
		const uchar *srcrow = src[y];
		QRgb *destrow = (QRgb*)dest.scanLine(y);
		for (int x = 0; x < src.cols; ++x) {

			destrow[x] = qRgba(srcrow[x], srcrow[x], srcrow[x], 255);
		}
	}
	return dest;
}

QImage Mat2QImage(const cv::Mat_<double> &src)
{
	double scale = 255.0;
	QImage dest(src.cols, src.rows, QImage::Format_ARGB32);
	for (int y = 0; y < src.rows; ++y) {
		const double *srcrow = src[y];
		QRgb *destrow = (QRgb*)dest.scanLine(y);
		for (int x = 0; x < src.cols; ++x) {
			unsigned int color = srcrow[x] * scale;
			destrow[x] = qRgba(color, color, color, 255);
		}
	}
	return dest;
}

void imshow(QLabel* const label, const QImage &qimg)          
{
	label->move(0,0);
	// display on label
	label->setPixmap(QPixmap::fromImage(qimg));
	// resize the label to fit the image
	label->resize(label->pixmap()->size());
}

Mat qimage2mat(const QImage& qimage) {
	cv::Mat mat = cv::Mat(qimage.height(), qimage.width(), CV_8UC4, (uchar*)qimage.bits(), qimage.bytesPerLine());
	cv::Mat mat2 = cv::Mat(mat.rows, mat.cols, CV_8UC3 );
	int from_to[] = { 0,0,  1,1,  2,2 };
	cv::mixChannels( &mat, 1, &mat2, 1, from_to, 3 );
	return mat2;
};

struct sort_pred {
	bool operator()(const std::pair<int,int> &left, const std::pair<int,int> &right) {
		return left.first > right.first;
	}
};

struct sort_double_pred {
	bool operator()(const std::pair<double,int> &left, const std::pair<double,int> &right) {
		return left.first > right.first;
	}
};

struct sort_double_increase {
	bool operator()(const std::pair<double,int> &left, const std::pair<double,int> &right) {
		return left.first < right.first;
	}
};

inline bool exists_test (const std::string& name) {
	struct stat buffer;   
	return (stat (name.c_str(), &buffer) == 0); 
}

bool isValid( int newX, int newY)
{
	if( newX >= 0 && newY >= 0 && newX < inputcols && newY < inputrows)
	{
		return true;
	}
	return false;
}

bool isValid_fullres( int newX, int newY)
{
	if( newX >= 0 && newY >= 0 && newX < inputcols_fullres && newY < inputrows_fullres)
	{
		return true;
	}
	return false;
}

bool isValid_MSL(int newX, int newY){
	if( newX >= 0 && newY >= 0 && newX < inputcols_fullres && newY < inputrows_fullres)
	{
		return true;
	}
	return false;
}

bool isValidbondary( int X, int Y, int l)
{
	if (l >= 0)
	{
		if( X >= regioninvalid_boundary_constraint[l][0] && Y >= regioninvalid_boundary_constraint[l][1] && X <= regioninvalid_boundary_constraint[l][2] && Y <= regioninvalid_boundary_constraint[l][3])
		{
			if (l == 2)
			{
				//qDebug()<<"l: "<<l<<", X: "<<X<<", Y: "<<Y<<"invalid region: "<<regioninvalid_boundary_constraint[l][0]<<regioninvalid_boundary_constraint[l][1]<<regioninvalid_boundary_constraint[l][2]<<regioninvalid_boundary_constraint[l][3];
			}
			return false;
		}
		return true;
	} 
	else
	{
		return true;
	}
}

bool isValidLocal(int X, int Y, int l){
	//if (l >= gcGuide_MSL_fullres(Y, X) * 5 && l < gcGuide_MSL_fullres(Y, X) * 5 + 5)
	//{
	//	return true;
	//} 
	//else
	//{
	//	return false;
	//}
	return true;
}

bool borderValid(int X_syn, int Y_syn, int X_input, int Y_input){
	if ((X_syn >= border && X_input < border) || (Y_syn >= border && Y_input < border) || (X_syn < syncols - border && X_input >= inputcols - border) || (Y_syn < synrows - border && Y_input >= inputrows - border))
	{
		//qDebug()<<"border invalid";
		return false;
	}
	return true;
}

bool borderValid_fullres(int X_syn, int Y_syn, int X_input, int Y_input){
	if ((X_syn >= border_fullres && X_input < border_fullres) || (Y_syn >= border_fullres && Y_input < border_fullres) || (X_syn < syncols_fullres - border_fullres && X_input >= inputcols_fullres - border_fullres) || (Y_syn < synrows_fullres - border_fullres && Y_input >= inputrows_fullres - border_fullres))
	{
		//qDebug()<<"border invalid";
		return false;
	}
	return true;
}

bool borderValid_MSL(int X_syn, int Y_syn, int X_input, int Y_input){
	if ((X_syn >= border_fullres && X_input < border_fullres) || (Y_syn >= border_fullres && Y_input < border_fullres) || (X_syn < syncols_fullres - border_fullres && X_input >= inputcols_fullres - border_fullres) || (Y_syn < synrows_fullres - border_fullres && Y_input >= inputrows_fullres - border_fullres))
	{
		return false;
	}
	return true;
}

vector<Point2i> point2Node(cv::Mat mat)
{
	vector<Point2i> nodes;

	for(int y = 0; y < mat.rows; y++)
	{
		for(int x = 0; x < mat.cols; x++)
		{
			nodes.push_back(Point2i(x,y));
		}
	}
	return nodes;
}

int dataFn(int p, int l)
{
	int newX = -gcLabels[l].x + gcNodes[p].x;
	int newY = -gcLabels[l].y + gcNodes[p].y;
	if( isValid(newX,newY) )
	{
		return 0;
	}
	return cost_data_inf;
}

int dataFnborderGuide(int p, int l){
	int newX = -gcLabels[l].x + gcNodes[p].x;
	int newY = -gcLabels[l].y + gcNodes[p].y;
	if( isValid(newX,newY) )
	{
		// see if this pixel satisfies border constraint
		if (borderValid(gcNodes[p].x, gcNodes[p].y, newX, newY))
		{
			if (gcGuide(gcNodes[p].y, gcNodes[p].x)!=l && gcGuide(gcNodes[p].y, gcNodes[p].x)!=-1)
			{
				return cost_data_guide_inf;
			} 
			else
			{
				return 0;
			}
		}
		return cost_data_inf;
	}
	return cost_data_inf;
}

int dataFnborderScribble(int p, int l){
	int newX = -gcLabels[l].x + gcNodes[p].x;
	int newY = -gcLabels[l].y + gcNodes[p].y;
	if( isValid(newX,newY) )
	{
		// see if this pixel satisfies border constraint
		if (borderValid(gcNodes[p].x, gcNodes[p].y, newX, newY))
		{
			if (gcGuideScribble(gcNodes[p].y, gcNodes[p].x) != gcLabel(newY, newX) && gcGuideScribble(gcNodes[p].y, gcNodes[p].x)!= 0)
			{
				return 100; // weakly penalized as the scribble may not be accurate
			} 
			else
			{
				return 0;
			}
		}
		return cost_data_inf;
	}
	return cost_data_inf;
}

int dataFnborderPrior(int p, int l){
	int newX = -gcLabels[l].x + gcNodes[p].x;
	int newY = -gcLabels[l].y + gcNodes[p].y;
	if( isValid(newX,newY) )
	{
		// see if this pixel satisfies border constraint
		if (borderValid(gcNodes[p].x, gcNodes[p].y, newX, newY))
		{

			if (gcLabel(newY, newX) == 0)
			{
				return globalnumRep * cost_data_prior_scaler;
			} 
			else
			{
				return (globalnumRep - (int)gcLabel(newY, newX)) * cost_data_prior_scaler;
			}

		}
		return cost_data_inf;
	}
	return cost_data_inf;
}

int dataFnSWborder(int p, int l){
	int newX = -gcLabels[l].x + gcNodes[p].x;
	int newY = -gcLabels[l].y + gcNodes[p].y;
	if( isValid(newX,newY) )
	{
		// see if this pixel satisfies border constraint
		if (borderValid(gcNodes[p].x, gcNodes[p].y, newX, newY))
		{
			// test guidance map
			if (gcGuide(gcNodes[p].y, gcNodes[p].x)!=l && gcGuide(gcNodes[p].y, gcNodes[p].x)!=-1)
			{
				return cost_data_guide_inf;
			} 
			else
			{
				return 0;
			}
		}
		return cost_data_inf;
	}
	return cost_data_inf;
}

int dataFnCooCborder(int p, int l){
	int newX = -gcLabels[l].x + gcNodes[p].x;
	int newY = -gcLabels[l].y + gcNodes[p].y;
	if( isValid(newX,newY) )
	{
		// see if this pixel satisfies border constraint
		if (borderValid(gcNodes[p].x, gcNodes[p].y, newX, newY))
		{
			if (gcPosGuide(gcNodes[p].y, gcNodes[p].x)!=-1){
				if (gcPosGuide(gcNodes[p].y, gcNodes[p].x)!= gcLabel(newY, newX))
				{
					return cost_data_guide_inf;
				} 
				else{
					return 0;
				}
			} 
			return 0;
		}
		return cost_data_inf;
	}
	return cost_data_inf;
}

int dataFnCooCborderNeg(int p, int l){
	int newX = -gcLabels[l].x + gcNodes[p].x;
	int newY = -gcLabels[l].y + gcNodes[p].y;
	if( isValid(newX,newY) )
	{
		// see if this pixel satisfies border constraint
		if (borderValid(gcNodes[p].x, gcNodes[p].y, newX, newY))
		{
			if (gcNegGuide(gcNodes[p].y, gcNodes[p].x)!=-1)
			{
				if (gcNegGuide(gcNodes[p].y, gcNodes[p].x) == l)
				{
					return cost_data_inf;
				} 
				else
				{
					return 0;
				}
			} 
			else if (gcPosGuide(gcNodes[p].y, gcNodes[p].x)!=-1){
				if (gcPosGuide(gcNodes[p].y, gcNodes[p].x)!= gcLabel(newY, newX))
				{
					return cost_data_inf;
				} 
				else{
					return 0;
				}
			} 
		}
		return cost_data_inf;
	}
	return cost_data_inf;
}

int dataFn2(int p, int l)
{
	int newX = -gcLabels[l].x + gcNodes[p].x;
	int newY = -gcLabels[l].y + gcNodes[p].y;
	if( isValid(newX,newY) )
	{
		// test guidance map
		if (gcGuide(gcNodes[p].y, gcNodes[p].x)!=l && gcGuide(gcNodes[p].y, gcNodes[p].x)!=-1)
		{
			return cost_data_guide_inf;
		} 
		else
		{
			return 0;
		}
		return 0;
	}
	return cost_data_inf;
}

int dataFn3(int p, int l){
	int cost = 0;
	// use the gcPosGuide and gcNegGuide
	int newX = -gcLabels[l].x + gcNodes[p].x;
	int newY = -gcLabels[l].y + gcNodes[p].y;
	if( isValid(newX,newY) )
	{
		if (gcNegGuide(gcNodes[p].y, gcNodes[p].x)!=-1)
		{
			if (gcNegGuide(gcNodes[p].y, gcNodes[p].x) == gcLabel(newY, newX))
			{
				//cost += cost_data_guide_inf/10;
			} 
			else
			{
				// encourage to replace the removed bb by another bb
				if (gcLabel(newY, newX) == 0)
				{
					//cost += cost_data_guide_inf/100;
				} 
				else
				{
					;
				}
			}
		} 
		else if (gcPosGuide(gcNodes[p].y, gcNodes[p].x)!=-1){
			if (gcPosGuide(gcNodes[p].y, gcNodes[p].x)!= gcLabel(newY, newX))
			{
				cost += cost_data_guide_inf/10;
			} 
		} 
	}
	else{
		cost = cost_data_inf;
	}
	return cost;
}

int dataFn4(int p, int l){
	int cost = 0;
	// use the gcPosGuide and gcNegGuide
	int newX = -gcLabels[l].x + gcNodes[p].x;
	int newY = -gcLabels[l].y + gcNodes[p].y;
	if( isValid(newX,newY) )
	{
		if (isValidbondary(gcNodes[p].x, gcNodes[p].y, gcLabel(newY, newX) - 1)) // should not pass shift label but bb label
		{
			if (gcPosGuide(gcNodes[p].y, gcNodes[p].x)!=-1){
				if (gcPosGuide(gcNodes[p].y, gcNodes[p].x)!= gcLabel(newY, newX))
				{
					cost += cost_data_guide_inf;
				} 
			}
			if (gcLabel(newY, newX) == 0){
				cost += cost_data_guide_inf/50;
			}


			//if (gcNegGuide(gcNodes[p].y, gcNodes[p].x)!=-1)
			//{
			//	if (gcNegGuide(gcNodes[p].y, gcNodes[p].x) == gcLabel(newY, newX))
			//	{
			//		//cost += cost_data_guide_inf/10;
			//	} 
			//	else
			//	{
			//		// encourage to replace the removed bb by another bb
			//		if (gcLabel(newY, newX) == 0)
			//		{
			//			//cost += cost_data_guide_inf/100;
			//		} 
			//		else
			//		{
			//			;
			//		}
			//	}
			//} 
			//else if (gcPosGuide(gcNodes[p].y, gcNodes[p].x)!=-1){
			//	if (gcPosGuide(gcNodes[p].y, gcNodes[p].x)!= gcLabel(newY, newX))
			//	{
			//		cost += cost_data_guide_inf/10;
			//	} 
			//} 
		} 
		else
		{
			cost = cost_data_inf * 10;
		}
	}
	else{
		cost = cost_data_inf;
	}
	return cost;
}

int smoothFn1(int p1, int p2, int l1, int l2)
{
	// to do: 
	// add label cost, start from no internal labels
	// add a switch to change mode from pixel to bb label
	// add internal bb label cost
	if(l1 == l2)
	{
		return 0;
	}

	int retMe = 0;

	Point2i x1_s_a = -gcLabels[l1] + gcNodes[p1];
	Point2i x2_s_b = -gcLabels[l2] + gcNodes[p2];

	if( isValid(x1_s_a.x, x1_s_a.y) && isValid(x2_s_b.x, x2_s_b.y) )
	{
		Point2i x1_s_b =  -gcLabels[l2] + gcNodes[p1];
		Point2i x2_s_a = -gcLabels[l1] + gcNodes[p2];

		if( isValid( x1_s_b.x, x1_s_b.y ) && isValid(x2_s_a.x, x2_s_a.y) )
		{
			int diff1 = gcImage(x1_s_a.y, x1_s_a.x) - gcImage(x1_s_b.y, x1_s_b.x);
			int diff2 = gcImage(x2_s_a.y, x2_s_a.x) - gcImage(x2_s_b.y, x2_s_b.x);
			retMe = round((double)cost_smooth_pixel_scaler*(sqrt((double)diff1*diff1) + sqrt((double)diff2*diff2)));
		}
		else
		{
			return cost_smooth_inf;
		}
		return retMe;
	}

	return cost_smooth_inf;	
}

int smoothFn3(int p1, int p2, int l1, int l2)
{
	// to do: 
	// add label cost, start from no internal labels
	// add a switch to change mode from pixel to bb label
	// add internal bb label cost
	if(l1 == l2)
	{
		return 0;
	}

	int retMe = 0;
	int retRep = 0;
	Point2i x1_s_a = -gcLabels[l1] + gcNodes[p1];
	Point2i x2_s_b = -gcLabels[l2] + gcNodes[p2];

	if( isValid(x1_s_a.x, x1_s_a.y) && isValid(x2_s_b.x, x2_s_b.y) )
	{
		Point2i x1_s_b =  -gcLabels[l2] + gcNodes[p1];
		Point2i x2_s_a = -gcLabels[l1] + gcNodes[p2];

		if( isValid( x1_s_b.x, x1_s_b.y ) && isValid(x2_s_a.x, x2_s_a.y) )
		{
			//double diff1 = gcImage(x1_s_a.y, x1_s_a.x) - gcImage(x1_s_b.y, x1_s_b.x);
			//double diff2 = gcImage(x2_s_a.y, x2_s_a.x) - gcImage(x2_s_b.y, x2_s_b.x);
			//double diffRep1 = 50 * (gcLabel(x1_s_a.y, x1_s_a.x) != gcLabel(x1_s_b.y, x1_s_b.x));
			//double diffRep2 = 50 * (gcLabel(x2_s_a.y, x2_s_a.x) != gcLabel(x2_s_b.y, x2_s_b.x));

			//// need to make sure the function is submodular
			//bool indicator = gcLabel(x1_s_a.y, x1_s_a.x) == gcLabel(x1_s_b.y, x1_s_b.x) && gcLabel(x2_s_a.y, x2_s_a.x) == gcLabel(x2_s_b.y, x2_s_b.x);
			//double diffRepinter1X = 20 * (gcLabelinterX(x1_s_a.y, x1_s_a.x) - gcLabelinterX(x1_s_b.y, x1_s_b.x));
			//double diffRepinter1Y = 20 * (gcLabelinterY(x1_s_a.y, x1_s_a.x) - gcLabelinterY(x1_s_b.y, x1_s_b.x));
			//double diffRepinter2X = 20 * (gcLabelinterX(x2_s_a.y, x2_s_a.x) - gcLabelinterX(x2_s_b.y, x2_s_b.x));
			//double diffRepinter2Y = 20 * (gcLabelinterY(x2_s_a.y, x2_s_a.x) - gcLabelinterY(x2_s_b.y, x2_s_b.x));

			//double energypixel = sqrt(double(diff1*diff1)) + sqrt(double(diff2*diff2));
			//double energylabel = sqrt(double(diffRep1*diffRep1)) + sqrt(double(diffRep2*diffRep2));
			//double energyinterlabel = indicator * (sqrt(double(diffRepinter1X*diffRepinter1X)) + sqrt(double(diffRepinter1Y*diffRepinter1Y)) + sqrt(double(diffRepinter2X * diffRepinter2X)) + sqrt(double(diffRepinter2Y * diffRepinter2Y)));
			//
			//retMe = round(energypixel + energylabel + energyinterlabel);
			
			int diff1 = gcImage(x1_s_a.y, x1_s_a.x) - gcImage(x1_s_b.y, x1_s_b.x);
			int diff2 = gcImage(x2_s_a.y, x2_s_a.x) - gcImage(x2_s_b.y, x2_s_b.x);
			double diffRep1 = 50 * (gcLabel(x1_s_a.y, x1_s_a.x) != gcLabel(x1_s_b.y, x1_s_b.x));
			double diffRep2 = 50 * (gcLabel(x2_s_a.y, x2_s_a.x) != gcLabel(x2_s_b.y, x2_s_b.x));

			bool indicator1 = gcLabel(x1_s_a.y, x1_s_a.x) == gcLabel(x1_s_b.y, x1_s_b.x);
			bool indicator2 = gcLabel(x2_s_a.y, x2_s_a.x) == gcLabel(x2_s_b.y, x2_s_b.x);
			double diffRepinter1X = 8 * (gcLabelinterX(x1_s_a.y, x1_s_a.x) - gcLabelinterX(x1_s_b.y, x1_s_b.x));
			double diffRepinter1Y = 8 * (gcLabelinterY(x1_s_a.y, x1_s_a.x) - gcLabelinterY(x1_s_b.y, x1_s_b.x));
			double diffRepinter2X = 8 * (gcLabelinterX(x2_s_a.y, x2_s_a.x) - gcLabelinterX(x2_s_b.y, x2_s_b.x));
			double diffRepinter2Y = 8 * (gcLabelinterY(x2_s_a.y, x2_s_a.x) - gcLabelinterY(x2_s_b.y, x2_s_b.x));

			double energypixel = sqrt(double(diff1*diff1)) + sqrt(double(diff2*diff2));
			double energylabel = sqrt(double(diffRep1*diffRep1)) + sqrt(double(diffRep2*diffRep2));
			double energyinterlabel = indicator1 * (sqrt(double(diffRepinter1X*diffRepinter1X)) + sqrt(double(diffRepinter1Y*diffRepinter1Y))) + indicator2 * (sqrt(double(diffRepinter2X * diffRepinter2X)) + sqrt(double(diffRepinter2Y * diffRepinter2Y)));

			retMe = ceil(energypixel + energylabel + energyinterlabel);
			//retMe = round(energyinterlabel);
		}
		else
		{
			return cost_smooth_inf;
		}
		return retMe;
	}

	return cost_smooth_inf;	
}

int dataFnborder(int p, int l){
	int newX = -gcLabels[l].x + gcNodes[p].x;
	int newY = -gcLabels[l].y + gcNodes[p].y;
	if( isValid(newX,newY) )
	{
		// see if this pixel satisfies border constraint
		if (borderValid(gcNodes[p].x, gcNodes[p].y, newX, newY))
		{
			return 0;
		}
		return cost_data_inf;
	}
	return cost_data_inf;
}

int dataFnborder_fullres(int p, int l){
	int newX = -gcLabels_fullres[l].x + gcNodes_fullres[p].x;
	int newY = -gcLabels_fullres[l].y + gcNodes_fullres[p].y;
	if(isValid_fullres(newX,newY))
	{
		// see if this pixel satisfies border constraint
		if (borderValid_fullres(gcNodes_fullres[p].x, gcNodes_fullres[p].y, newX, newY))
		{

			// see if this label is within the local move
			// this requires to find the label of the old assignment
			return 0;

		}
		return cost_data_inf;
	}
	return cost_data_inf;
}

int dataFnborder_MSL(int p, int l){
	int newX = -gcLabels_fullres[l].x + gcNodes_fullres[p].x;
	int newY = -gcLabels_fullres[l].y + gcNodes_fullres[p].y;
	if(isValid_MSL(newX,newY))
	{
		//// see if this pixel satisfies border constraint
		if (borderValid_MSL(gcNodes_fullres[p].x, gcNodes_fullres[p].y, newX, newY))
		{
			if (isValidLocal(gcNodes_fullres[p].x, gcNodes_fullres[p].y, l))
			{
				return 0;
			} 
			else
			{
				return cost_data_inf;
			}
		}
		return cost_data_inf;
	}
	return cost_data_inf;	
}

int smoothFn1_fullres(int p1, int p2, int l1, int l2){
	if(l1 == l2)
	{
		return 0;
	}

	int retMe = 0;

	Point2i x1_s_a = -gcLabels_fullres[l1] + gcNodes_fullres[p1];
	Point2i x2_s_b = -gcLabels_fullres[l2] + gcNodes_fullres[p2];

	if( isValid_fullres(x1_s_a.x, x1_s_a.y) && isValid_fullres(x2_s_b.x, x2_s_b.y) )
	{
		Point2i x1_s_b =  -gcLabels_fullres[l2] + gcNodes_fullres[p1];
		Point2i x2_s_a = -gcLabels_fullres[l1] + gcNodes_fullres[p2];

		if( isValid_fullres( x1_s_b.x, x1_s_b.y ) && isValid_fullres(x2_s_a.x, x2_s_a.y) )
		{
			int diff1 = gcImage_fullres(x1_s_a.y, x1_s_a.x) - gcImage_fullres(x1_s_b.y, x1_s_b.x);
			int diff2 = gcImage_fullres(x2_s_a.y, x2_s_a.x) - gcImage_fullres(x2_s_b.y, x2_s_b.x);
			retMe = round((double)cost_smooth_pixel_scaler*(sqrt((double)diff1*diff1) + sqrt((double)diff2*diff2)));
		}
		else
		{
			return cost_smooth_inf;
		}
		return retMe;
	}

	return cost_smooth_inf;	
}

int smoothFn1_MSL(int p1, int p2, int l1, int l2){
	if(l1 == l2)
	{
		return 0;
	}
	Point2i x1_s_a = -gcLabels_fullres[l1] + gcNodes_fullres[p1];
	Point2i x2_s_b = -gcLabels_fullres[l2] + gcNodes_fullres[p2];

	if(isValid_MSL(x1_s_a.x, x1_s_a.y) && isValid_MSL(x2_s_b.x, x2_s_b.y) && isValidLocal(gcNodes_fullres[p1].x, gcNodes_fullres[p1].y, l1) && isValidLocal(gcNodes_fullres[p2].x, gcNodes_fullres[p2].y, l2))
	{
		Point2i x1_s_b = -gcLabels_fullres[l2] + gcNodes_fullres[p1];
		Point2i x2_s_a = -gcLabels_fullres[l1] + gcNodes_fullres[p2];

		if(isValid_MSL(x1_s_b.x, x1_s_b.y) && isValid_MSL(x2_s_a.x, x2_s_a.y) && isValidLocal(gcNodes_fullres[p1].x, gcNodes_fullres[p1].y, l2) && isValidLocal(gcNodes_fullres[p2].x, gcNodes_fullres[p2].y, l1))
		{
			int diff1 = gcImage_fullres(x1_s_a.y, x1_s_a.x) - gcImage_fullres(x1_s_b.y, x1_s_b.x);
			int diff2 = gcImage_fullres(x2_s_a.y, x2_s_a.x) - gcImage_fullres(x2_s_b.y, x2_s_b.x);
			return round((double)cost_smooth_pixel_scaler*(sqrt((double)diff1*diff1) + sqrt((double)diff2*diff2)));
		}
		else
		{
			return cost_smooth_inf;
		}
	}
	return cost_smooth_inf;	
}

int smoothFn3_MSL(int p1, int p2, int l1, int l2)
{
	// to do: 
	// add label cost, start from no internal labels
	// add a switch to change mode from pixel to bb label
	// add internal bb label cost
	if(l1 == l2)
	{
		return 0;
	}

	int retMe = 0;
	int retRep = 0;
	Point2i x1_s_a = -gcLabels_fullres[l1] + gcNodes_fullres[p1];
	Point2i x2_s_b = -gcLabels_fullres[l2] + gcNodes_fullres[p2];

	if(isValid_MSL(x1_s_a.x, x1_s_a.y) && isValid_MSL(x2_s_b.x, x2_s_b.y) && isValidLocal(gcNodes_fullres[p1].x, gcNodes_fullres[p1].y, l1) && isValidLocal(gcNodes_fullres[p2].x, gcNodes_fullres[p2].y, l2))
	{
		Point2i x1_s_b = -gcLabels_fullres[l2] + gcNodes_fullres[p1];
		Point2i x2_s_a = -gcLabels_fullres[l1] + gcNodes_fullres[p2];

		if(isValid_MSL(x1_s_b.x, x1_s_b.y) && isValid_MSL(x2_s_a.x, x2_s_a.y) && isValidLocal(gcNodes_fullres[p1].x, gcNodes_fullres[p1].y, l2) && isValidLocal(gcNodes_fullres[p2].x, gcNodes_fullres[p2].y, l1))
		{
			int diff1 = gcImage_fullres(x1_s_a.y, x1_s_a.x) - gcImage_fullres(x1_s_b.y, x1_s_b.x);
			int diff2 = gcImage_fullres(x2_s_a.y, x2_s_a.x) - gcImage_fullres(x2_s_b.y, x2_s_b.x);
			double diffRep1 = 50 * (gcLabel_fullres(x1_s_a.y, x1_s_a.x) != gcLabel_fullres(x1_s_b.y, x1_s_b.x));
			double diffRep2 = 50 * (gcLabel_fullres(x2_s_a.y, x2_s_a.x) != gcLabel_fullres(x2_s_b.y, x2_s_b.x));

			bool indicator = gcLabel_fullres(x1_s_a.y, x1_s_a.x) == gcLabel_fullres(x1_s_b.y, x1_s_b.x) && gcLabel_fullres(x2_s_a.y, x2_s_a.x) == gcLabel_fullres(x2_s_b.y, x2_s_b.x);
			double diffRepinter1X = 4 * (gcLabelinterX_fullres(x1_s_a.y, x1_s_a.x) - gcLabelinterX_fullres(x1_s_b.y, x1_s_b.x));
			double diffRepinter1Y = 4 * (gcLabelinterY_fullres(x1_s_a.y, x1_s_a.x) - gcLabelinterY_fullres(x1_s_b.y, x1_s_b.x));
			double diffRepinter2X = 4 * (gcLabelinterX_fullres(x2_s_a.y, x2_s_a.x) - gcLabelinterX_fullres(x2_s_b.y, x2_s_b.x));
			double diffRepinter2Y = 4 * (gcLabelinterY_fullres(x2_s_a.y, x2_s_a.x) - gcLabelinterY_fullres(x2_s_b.y, x2_s_b.x));

			double energypixel = sqrt(double(diff1*diff1)) + sqrt(double(diff2*diff2));
			double energylabel = sqrt(double(diffRep1*diffRep1)) + sqrt(double(diffRep2*diffRep2));
			double energyinterlabel = indicator * (sqrt(double(diffRepinter1X*diffRepinter1X)) + sqrt(double(diffRepinter1Y*diffRepinter1Y)) + sqrt(double(diffRepinter2X * diffRepinter2X)) + sqrt(double(diffRepinter2Y * diffRepinter2Y)));

			//retMe = round(energypixel + energylabel);
			//retMe = round(energypixel + energylabel + energyinterlabel);
			//qDebug()<<"energypixel: "<<energypixel<<", energyinterlabel: "<<energyinterlabel;
			retMe = energypixel + energylabel;
		}
		else
		{
			return cost_smooth_inf;
		}
		return retMe;
	}

	return cost_smooth_inf;	
}

int smoothFnConstellation(int p1, int p2, int l1, int l2){

	if(l1 == l2)
	{
		return 0;
	}

	int retMe = 0;
	int retRep = 0;
	Point2i x1_s_a = -gcLabels[l1] + gcNodes[p1];
	Point2i x2_s_b = -gcLabels[l2] + gcNodes[p2];

	if( isValid(x1_s_a.x, x1_s_a.y) && isValid(x2_s_b.x, x2_s_b.y) )
	{
		Point2i x1_s_b =  -gcLabels[l2] + gcNodes[p1];
		Point2i x2_s_a = -gcLabels[l1] + gcNodes[p2];

		if( isValid( x1_s_b.x, x1_s_b.y ) && isValid(x2_s_a.x, x2_s_a.y) )
		{
			int diff1 = gcImage(x1_s_a.y, x1_s_a.x) - gcImage(x1_s_b.y, x1_s_b.x);
			int diff2 = gcImage(x2_s_a.y, x2_s_a.x) - gcImage(x2_s_b.y, x2_s_b.x);

			int idx1 = x1_s_a.y * inputcols + x1_s_a.x;
			int idx2 = x1_s_b.y * inputcols + x1_s_b.x;
			int idx3 = x2_s_a.y * inputcols + x2_s_a.x;
			int idx4 = x2_s_b.y * inputcols + x2_s_b.x;

			int diffCons1 = M_constellation(idx1, idx2);
			int diffCons2 = M_constellation(idx3, idx4);

			retMe = round(cost_smooth_pixel_scaler * (diff1*diff1 + diff2*diff2) + cost_smooth_constellation_scaler * (diffCons1 + diffCons2));
		}
		else
		{
			return cost_smooth_inf;
		}
		return retMe;
	}
	return cost_smooth_inf;	
}

int smoothFnConstellation2(int p1, int p2, int l1, int l2){

	if(l1 == l2)
	{
		return 0;
	}

	int retMe = 0;
	int retRep = 0;
	Point2i x1_s_a = -gcLabels[l1] + gcNodes[p1];
	Point2i x2_s_b = -gcLabels[l2] + gcNodes[p2];

	if( isValid(x1_s_a.x, x1_s_a.y) && isValid(x2_s_b.x, x2_s_b.y) )
	{
		Point2i x1_s_b =  -gcLabels[l2] + gcNodes[p1];
		Point2i x2_s_a = -gcLabels[l1] + gcNodes[p2];

		if( isValid( x1_s_b.x, x1_s_b.y ) && isValid(x2_s_a.x, x2_s_a.y) )
		{
			int diff1 = gcImage(x1_s_a.y, x1_s_a.x) - gcImage(x1_s_b.y, x1_s_b.x);
			int diff2 = gcImage(x2_s_a.y, x2_s_a.x) - gcImage(x2_s_b.y, x2_s_b.x);

			int idx1 = x1_s_a.y * inputcols + x1_s_a.x;
			int idx2 = x1_s_b.y * inputcols + x1_s_b.x;
			int idx3 = x2_s_a.y * inputcols + x2_s_a.x;
			int idx4 = x2_s_b.y * inputcols + x2_s_b.x;

			int diffCons1 = M_constellation(idx1, idx2);
			int diffCons2 = M_constellation(idx3, idx4);

			int diffRep1 = cost_smooth_label*(gcLabel(x1_s_a.y, x1_s_a.x) != gcLabel(x1_s_b.y, x1_s_b.x));
			int diffRep2 = cost_smooth_label*(gcLabel(x2_s_a.y, x2_s_a.x) != gcLabel(x2_s_b.y, x2_s_b.x));

			//int diffRepinter1X = cost_smooth_label*(gcLabel(x1_s_a.y, x1_s_a.x) == gcLabel(x1_s_b.y, x1_s_b.x)) * (gcLabelinterX(x1_s_a.y, x1_s_a.x) - gcLabelinterX(x1_s_b.y, x1_s_b.x));
			//int diffRepinter1Y = cost_smooth_label*(gcLabel(x1_s_a.y, x1_s_a.x) == gcLabel(x1_s_b.y, x1_s_b.x)) * (gcLabelinterY(x1_s_a.y, x1_s_a.x) - gcLabelinterY(x1_s_b.y, x1_s_b.x));
			//int diffRepinter2X = cost_smooth_label*(gcLabel(x2_s_a.y, x2_s_a.x) == gcLabel(x2_s_b.y, x2_s_b.x)) * (gcLabelinterX(x2_s_a.y, x2_s_a.x) - gcLabelinterX(x2_s_b.y, x2_s_b.x));
			//int diffRepinter2Y = cost_smooth_label*(gcLabel(x2_s_a.y, x2_s_a.x) == gcLabel(x2_s_b.y, x2_s_b.x)) * (gcLabelinterY(x2_s_a.y, x2_s_a.x) - gcLabelinterY(x2_s_b.y, x2_s_b.x));

			int diffRepinter1X = 0;
			int diffRepinter1Y = 0;
			int diffRepinter2X = 0;
			int diffRepinter2Y = 0;

			retMe = round(cost_smooth_pixel_scaler * (diff1*diff1 + diff2*diff2 + diffRep1*diffRep1 + diffRep2*diffRep2 + diffRepinter1X*diffRepinter1X + diffRepinter1Y*diffRepinter1Y + diffRepinter2X * diffRepinter2X + diffRepinter2Y * diffRepinter2Y) + cost_smooth_constellation_scaler * (diffCons1 + diffCons2));
		}
		else
		{
			return cost_smooth_inf;
		}
		return retMe;
	}
	return cost_smooth_inf;	
}

int smoothFnConstellation3(int p1, int p2, int l1, int l2){

	if(l1 == l2)
	{
		return 0;
	}

	int retMe = 0;
	int retRep = 0;
	Point2i x1_s_a = -gcLabels[l1] + gcNodes[p1];
	Point2i x2_s_b = -gcLabels[l2] + gcNodes[p2];

	if( isValid(x1_s_a.x, x1_s_a.y) && isValid(x2_s_b.x, x2_s_b.y) )
	{
		Point2i x1_s_b =  -gcLabels[l2] + gcNodes[p1];
		Point2i x2_s_a = -gcLabels[l1] + gcNodes[p2];

		if( isValid( x1_s_b.x, x1_s_b.y ) && isValid(x2_s_a.x, x2_s_a.y) )
		{
			int diff1 = gcImage(x1_s_a.y, x1_s_a.x) - gcImage(x1_s_b.y, x1_s_b.x);
			int diff2 = gcImage(x2_s_a.y, x2_s_a.x) - gcImage(x2_s_b.y, x2_s_b.x);

			int idx1 = x1_s_a.y * inputcols + x1_s_a.x;
			int idx2 = x1_s_b.y * inputcols + x1_s_b.x;
			int idx3 = x2_s_a.y * inputcols + x2_s_a.x;
			int idx4 = x2_s_b.y * inputcols + x2_s_b.x;

			double diffRep1 = 50 * (gcLabel(x1_s_a.y, x1_s_a.x) != gcLabel(x1_s_b.y, x1_s_b.x));
			double diffRep2 = 50 * (gcLabel(x2_s_a.y, x2_s_a.x) != gcLabel(x2_s_b.y, x2_s_b.x));

			int diffCons1 = M_constellation(idx1, idx2);
			int diffCons2 = M_constellation(idx3, idx4);

			double energypixel = sqrt(double(diff1*diff1)) + sqrt(double(diff2*diff2));
			double energylabel = sqrt(double(diffRep1*diffRep1)) + sqrt(double(diffRep2*diffRep2));
			double energycons = sqrt(double(diffCons1*diffCons1)) + sqrt(double(diffCons2*diffCons2));

			//qDebug()<<energypixel<<energylabel<<energycons;

			retMe = round(energypixel + energylabel + energycons);
			//retMe = round(energycons);
		}
		else
		{
			return cost_smooth_inf;
		}
		return retMe;
	}
	return cost_smooth_inf;	
}

int smoothFn(int p1, int p2, int l1, int l2)
{
	//if ( (l1-l2)*(l1-l2) <= 4 ) return((l1-l2)*(l1-l2));
	//else return(4);
	if (l1 == l2)
	{
		return 0;
	} 
	else
	{
		return 1;
	}
	
}

void ImageViewer::synSwitchMode1(){
	if (!flag_scribble && !flag_interactive)
	{
        synmode = 1;
	}

	if (flag_scribble)
	{
        synmode = 3;
	}

	if (flag_interactive)
	{
        synmode = 2;
	}	
	if (flag_MS)
	{
	    synSampleMSL();
	} 
	else
	{
		synSample();
	}

}

void ImageViewer::synSwitchMSL(){
	if (flag_MS)
	{
		flag_MS = false;
		qDebug()<<flag_MS;
		synSample();
	}
	else
	{
		flag_MS = true;
		qDebug()<<flag_MS;
		synSampleMSL();
	}
}

void ImageViewer::IniImg(){

	scalerSynX = 1;
	scalerSynY = 1;
	renderRepLabelflag = 0;

	rep = new Rep;
	rep->qimgInputGray_fullres = new QImage;
	rep->qimgInputGray_scaled = new QImage;
	rep->qlabelInputGray_fullres = new QLabel;
	rep->qlabelInputGray_scaled = new QLabel;
	rep->qimgInputlabel_fullres = new QImage;
	rep->qimgInputlabel_scaled = new QImage;
	rep->qimgInputlabelinterX_fullres = new QImage; 
	rep->qimgInputlabelinterX_scaled = new QImage;
	rep->qimgInputlabelinterY_fullres = new QImage;
	rep->qimgInputlabelinterY_scaled = new QImage;

	rep->qimgSyn_fullres = new QImage;
	rep->qimgSyn_scaled = new QImage;
	rep->qimgSynGray_fullres = new QImage;
	rep->qimgSynGray_scaled = new QImage;
	rep->qlabelSyn_fullres = new QLabel;
	rep->qlabelSyn_scaled = new QLabel;
	rep->qlabelSynGray_fullres = new QLabel;
	rep->qlabelSynGray_scaled = new QLabel;
	rep->qimgSynlabelColor_fullres = new QImage;
	rep->qlabelSynlabelColor_fullres = new QLabel;

	rep->qimgUserGuide_scaled = new QImage;
	rep->qimgUserGuideGray_fullres = new QImage;
	rep->qimgUserGuideGray_scaled = new QImage;

	rep->qimgInput_fullres = new QImage(filename_imgInput);
	rep->qlabelInput_fullres =  new QLabel;
	rep->qlabelInput_fullres->setPixmap(QPixmap::fromImage(*rep->qimgInput_fullres));
	rep->rowsInput_fullres = rep->qimgInput_fullres->height();
	rep->colsInput_fullres = rep->qimgInput_fullres->width();
	rep->qimgInput_scaled = new QImage;
	*rep->qimgInput_scaled  = rep->qimgInput_fullres->scaled(rep->qimgInput_fullres->size() * scalerRes, Qt::KeepAspectRatio, Qt::SmoothTransformation);
	rep->qlabelInput_scaled =  new QLabel;
	rep->qlabelInput_scaled->setPixmap(QPixmap::fromImage(*rep->qimgInput_scaled));

	// matrix for (fullres and scaled) grayscale input image
	rep->imgInput_fullres = qimage2mat(*rep->qimgInput_fullres);
	cvtColor(rep->imgInput_fullres, rep->imgInputGray_fullres, CV_BGR2GRAY);
	rep->imgInput_scaled = qimage2mat(*rep->qimgInput_scaled);
	cvtColor(rep->imgInput_scaled, rep->imgInputGray_scaled, CV_BGR2GRAY);
	gcImage = rep->imgInputGray_scaled;
	gcImage_fullres = rep->imgInputGray_fullres;
	rep->rowsInput_fullres = rep->imgInputGray_fullres.rows;
	rep->colsInput_fullres = rep->imgInputGray_fullres.cols;
	rep->rowsInput_scaled = rep->imgInputGray_scaled.rows;
	rep->colsInput_scaled = rep->imgInputGray_scaled.cols;
	global_rowsInput_fullres = rep->rowsInput_fullres;
	global_colsInput_fullres = rep->colsInput_fullres;
	global_rowsInput_scaled = rep->rowsInput_scaled;
	global_colsInput_scaled = rep->colsInput_scaled;

	gcNodesInput = point2Node(rep->imgInputGray_scaled);

	// initialize synthesis image
	rep->list_shiftX_scaled.push_back(0);
	rep->list_shiftY_scaled.push_back(0);
	rep->list_shiftX_fullres.push_back(0);
	rep->list_shiftY_fullres.push_back(0);

	rep->colsSyn_scaled = rep->colsInput_scaled;
	rep->rowsSyn_scaled = rep->rowsInput_scaled;
	rep->colsSyn_fullres = rep->colsInput_fullres;
	rep->rowsSyn_fullres = rep->rowsInput_fullres;

	rep->num_shiftX = 1;
	rep->num_shiftY = 1;

	// user guidance
	//if (exists_test(filename_userguideInput.toUtf8().constData()))
	//{
	//	rep->qimgUserGuide_fullres = new QImage(filename_userguideInput);
	//} 
	//else
	//{
	//	rep->qimgUserGuide_fullres = new QImage(rep->colsInput_fullres, rep->rowsInput_fullres, QImage::Format_RGB32);
	//	rep->qimgUserGuide_fullres->fill(QColor(Qt::white).rgb());
	//}

	//*rep->qimgUserGuide_scaled = rep->qimgUserGuide_fullres->scaled(rep->qimgUserGuide_fullres->size() * scalerRes, Qt::KeepAspectRatio, Qt::FastTransformation);
	//rep->imgUserGuide_fullres = qimage2mat(*rep->qimgUserGuide_fullres);
	//cvtColor(rep->imgUserGuide_fullres, rep->imgUserGuideGray_fullres, CV_BGR2GRAY);
	//rep->imgUserGuide_scaled = qimage2mat(*rep->qimgUserGuide_scaled);
	//cvtColor(rep->imgUserGuide_scaled, rep->imgUserGuideGray_scaled, CV_BGR2GRAY);
	//gcUserGuide = Mat1d::zeros(rep->imgUserGuideGray_scaled.size());
	//cv::threshold(255 - rep->imgUserGuideGray_scaled, gcUserGuide, 100, 255, cv::THRESH_BINARY); // this to be changed to account for different bb lables




}

void ImageViewer::IniRep(){
	// read in fullres rep
	QFile fileRep(filename_repInput);
	if(fileRep.open(QIODevice::ReadOnly))
	{
		QTextStream in(&fileRep);
		in >> rep->numRep;
		globalnumRep = rep->numRep;
		rep->sizeRep.resize(rep->numRep);
		rep->numBB = 0;
		for (int i_rep = 0; i_rep < rep->numRep; i_rep++)
		{
			in>>rep->sizeRep[i_rep];
			rep->numBB += rep->sizeRep[i_rep];
		}
		cur_num_KNN = min(rep->numBB, ini_num_KNN);


		rep->repX_fullres.resize(rep->numRep);
		rep->repY_fullres.resize(rep->numRep);
		rep->repW_fullres.resize(rep->numRep);
		rep->repH_fullres.resize(rep->numRep);
		for (int i_rep = 0; i_rep < rep->numRep; i_rep++)
		{
			rep->repX_fullres[i_rep].resize(rep->sizeRep[i_rep]);
			rep->repY_fullres[i_rep].resize(rep->sizeRep[i_rep]);
			rep->repW_fullres[i_rep].resize(rep->sizeRep[i_rep]);
			rep->repH_fullres[i_rep].resize(rep->sizeRep[i_rep]);
			for (int j_rep = 0; j_rep < rep->sizeRep[i_rep]; j_rep++)
			{
				in>>rep->repX_fullres[i_rep][j_rep];
				in>>rep->repY_fullres[i_rep][j_rep];
				in>>rep->repW_fullres[i_rep][j_rep];
				in>>rep->repH_fullres[i_rep][j_rep];
			}
		}
	}

	// scale the input labels
	rep->repX_scaled.resize(rep->numRep);
	rep->repY_scaled.resize(rep->numRep);
	rep->repW_scaled.resize(rep->numRep);
	rep->repH_scaled.resize(rep->numRep);
	for (int i_rep = 0; i_rep < rep->numRep; i_rep++)
	{
		rep->repX_scaled[i_rep].resize(rep->sizeRep[i_rep]);
		rep->repY_scaled[i_rep].resize(rep->sizeRep[i_rep]);
		rep->repW_scaled[i_rep].resize(rep->sizeRep[i_rep]);
		rep->repH_scaled[i_rep].resize(rep->sizeRep[i_rep]);
		for (int j_rep = 0; j_rep < rep->sizeRep[i_rep]; j_rep++)
		{
			// scale and make sure the box does not exceed image boundary
			rep->repX_scaled[i_rep][j_rep] = min(round(rep->repX_fullres[i_rep][j_rep] * scalerRes), rep->colsInput_scaled - 1);
			rep->repY_scaled[i_rep][j_rep] = min(round(rep->repY_fullres[i_rep][j_rep] * scalerRes), rep->rowsInput_scaled - 1);
			rep->repW_scaled[i_rep][j_rep] = max(1, min(round((rep->repX_fullres[i_rep][j_rep] + rep->repW_fullres[i_rep][j_rep]) * scalerRes) - rep->repX_scaled[i_rep][j_rep], rep->colsInput_scaled - 1 - rep->repX_scaled[i_rep][j_rep]));
			rep->repH_scaled[i_rep][j_rep] = max(1, min(round((rep->repY_fullres[i_rep][j_rep] + rep->repH_fullres[i_rep][j_rep]) * scalerRes) - rep->repY_scaled[i_rep][j_rep], rep->rowsInput_scaled - 1 - rep->repY_scaled[i_rep][j_rep]));
		}
	}

	// make fullres maps and scaled maps for rep labels
	rep->imgInputlabel_scaled = Mat1b::zeros(rep->rowsInput_scaled, rep->colsInput_scaled); 
	for (int i_rep = 0; i_rep < rep->numRep; i_rep++)
	{
		for (int j_rep = 0; j_rep < rep->sizeRep[i_rep]; j_rep++)
		{
			rep->imgInputlabel_scaled(Range(rep->repY_scaled[i_rep][j_rep], rep->repY_scaled[i_rep][j_rep] + rep->repH_scaled[i_rep][j_rep]), Range(rep->repX_scaled[i_rep][j_rep], rep->repX_scaled[i_rep][j_rep] + rep->repW_scaled[i_rep][j_rep])) = (i_rep + 1);
		}
	}
	gcLabel = rep->imgInputlabel_scaled;

	// make maps for rep internal labels
	rep->imgInputlabelinterX_scaled = Mat1d::zeros(rep->rowsInput_scaled, rep->colsInput_scaled); 
	rep->imgInputlabelinterY_scaled = Mat1d::zeros(rep->rowsInput_scaled, rep->colsInput_scaled); 
	for (int i_rep = 0; i_rep < rep->numRep; i_rep++)
	{
		for (int j_rep = 0; j_rep < rep->sizeRep[i_rep]; j_rep++)
		{
			cv::Mat1d X, Y;
			meshgridTest(cv::Range(0,rep->repW_scaled[i_rep][j_rep]- 1), cv::Range(0, rep->repH_scaled[i_rep][j_rep] - 1), X, Y); 
			X = X * ((double)1/(double)max(1, rep->repW_scaled[i_rep][j_rep]));
			Y = Y * ((double)1/(double)max(1, rep->repH_scaled[i_rep][j_rep]));
			X.copyTo(rep->imgInputlabelinterX_scaled(Range(rep->repY_scaled[i_rep][j_rep], rep->repY_scaled[i_rep][j_rep] + rep->repH_scaled[i_rep][j_rep]), Range(rep->repX_scaled[i_rep][j_rep], rep->repX_scaled[i_rep][j_rep] + rep->repW_scaled[i_rep][j_rep])));
			Y.copyTo(rep->imgInputlabelinterY_scaled(Range(rep->repY_scaled[i_rep][j_rep], rep->repY_scaled[i_rep][j_rep] + rep->repH_scaled[i_rep][j_rep]), Range(rep->repX_scaled[i_rep][j_rep], rep->repX_scaled[i_rep][j_rep] + rep->repW_scaled[i_rep][j_rep])));
		}
	}
	gcLabelinterX = rep->imgInputlabelinterX_scaled;
	gcLabelinterY = rep->imgInputlabelinterY_scaled;

	rep->imgInputlabel_fullres = Mat1b::zeros(rep->rowsInput_fullres, rep->colsInput_fullres); //or, rep->imgSynGray_scaled.create(rows, cols);
	for (int i_rep = 0; i_rep < rep->numRep; i_rep++)
	{
		for (int j_rep = 0; j_rep < rep->sizeRep[i_rep]; j_rep++)
		{
			rep->repX_fullres[i_rep][j_rep] = max(0, min(rep->repX_fullres[i_rep][j_rep], rep->colsInput_fullres - 1));
			rep->repY_fullres[i_rep][j_rep] = max(0, min(rep->repY_fullres[i_rep][j_rep], rep->rowsInput_fullres - 1));
			rep->repW_fullres[i_rep][j_rep] = max(1, min((rep->repX_fullres[i_rep][j_rep] + rep->repW_fullres[i_rep][j_rep]) - rep->repX_fullres[i_rep][j_rep], rep->colsInput_fullres - 1 - rep->repX_fullres[i_rep][j_rep]));
			rep->repH_fullres[i_rep][j_rep] = max(1, min((rep->repY_fullres[i_rep][j_rep] + rep->repH_fullres[i_rep][j_rep]) - rep->repY_fullres[i_rep][j_rep], rep->rowsInput_fullres - 1 - rep->repY_fullres[i_rep][j_rep]));
			rep->imgInputlabel_fullres(Range(rep->repY_fullres[i_rep][j_rep], rep->repY_fullres[i_rep][j_rep] + rep->repH_fullres[i_rep][j_rep]), Range(rep->repX_fullres[i_rep][j_rep], rep->repX_fullres[i_rep][j_rep] + rep->repW_fullres[i_rep][j_rep])) = (i_rep + 1);
		}
	}

	gcLabel_fullres = rep->imgInputlabel_fullres;

	// make maps for rep internal labels
	rep->imgInputlabelinterX_fullres = Mat1d::zeros(rep->rowsInput_fullres, rep->colsInput_fullres); 
	rep->imgInputlabelinterY_fullres = Mat1d::zeros(rep->rowsInput_fullres, rep->colsInput_fullres); 
	for (int i_rep = 0; i_rep < rep->numRep; i_rep++)
	{
		for (int j_rep = 0; j_rep < rep->sizeRep[i_rep]; j_rep++)
		{
			cv::Mat1d X, Y;
			meshgridTest(cv::Range(0,rep->repW_fullres[i_rep][j_rep]- 1), cv::Range(0, rep->repH_fullres[i_rep][j_rep] - 1), X, Y); 
			X = X * ((double)1/(double)max(1, rep->repW_fullres[i_rep][j_rep]));
			Y = Y * ((double)1/(double)max(1, rep->repH_fullres[i_rep][j_rep]));
			X.copyTo(rep->imgInputlabelinterX_fullres(Range(rep->repY_fullres[i_rep][j_rep], rep->repY_fullres[i_rep][j_rep] + rep->repH_fullres[i_rep][j_rep]), Range(rep->repX_fullres[i_rep][j_rep], rep->repX_fullres[i_rep][j_rep] + rep->repW_fullres[i_rep][j_rep])));
			Y.copyTo(rep->imgInputlabelinterY_fullres(Range(rep->repY_fullres[i_rep][j_rep], rep->repY_fullres[i_rep][j_rep] + rep->repH_fullres[i_rep][j_rep]), Range(rep->repX_fullres[i_rep][j_rep], rep->repX_fullres[i_rep][j_rep] + rep->repW_fullres[i_rep][j_rep])));
		}
	}
	gcLabelinterX_fullres = rep->imgInputlabelinterX_fullres;
	gcLabelinterY_fullres = rep->imgInputlabelinterY_fullres;

	ini_label = 0;
}

void ImageViewer::IniCooC(){
	QFile fileCooC(filename_coocInput);
	if(fileCooC.open(QIODevice::ReadOnly))
	{
		QTextStream in(&fileCooC);
		in >> rep->numCooC;

		rep->sizeCooC.resize(rep->numCooC);
		for (int i_rep = 0; i_rep < rep->numCooC; i_rep++)
		{
			in>>rep->sizeCooC[i_rep];
		}

		rep->coocX_fullres.resize(rep->numCooC);
		rep->coocY_fullres.resize(rep->numCooC);

		for (int i_rep = 0; i_rep < rep->numCooC; i_rep++)
		{
			rep->coocX_fullres[i_rep].resize(rep->sizeCooC[i_rep]);
			rep->coocY_fullres[i_rep].resize(rep->sizeCooC[i_rep]);
			for (int j_rep = 0; j_rep < rep->sizeCooC[i_rep]; j_rep++)
			{
				in>>rep->coocX_fullres[i_rep][j_rep];
				in>>rep->coocY_fullres[i_rep][j_rep];
			}
		}
	}

	// scale the input labels
	rep->coocX_scaled.resize(rep->numCooC);
	rep->coocY_scaled.resize(rep->numCooC);
	for (int i_rep = 0; i_rep < rep->numCooC; i_rep++)
	{
		rep->coocX_scaled[i_rep].resize(rep->sizeCooC[i_rep]);
		rep->coocY_scaled[i_rep].resize(rep->sizeCooC[i_rep]);
		for (int j_rep = 0; j_rep < rep->sizeCooC[i_rep]; j_rep++)
		{
			// scale and make sure the box does not exceed image boundary
			rep->coocX_scaled[i_rep][j_rep] = round(rep->coocX_fullres[i_rep][j_rep] * scalerRes);
			rep->coocY_scaled[i_rep][j_rep] = round(rep->coocY_fullres[i_rep][j_rep] * scalerRes);
		}
	}
}

void ImageViewer::IniCooCCross(){
	QFile fileCooC(filename_coocInput);
	if(fileCooC.open(QIODevice::ReadOnly))
	{
		QTextStream in(&fileCooC);
		in >> rep->numCooC;

		rep->sizeCooC.resize(rep->numCooC);
		for (int i_rep = 0; i_rep < rep->numCooC; i_rep++)
		{
			in>>rep->sizeCooC[i_rep];
		}

		rep->coocX_fullres.resize(rep->numCooC);
		rep->coocY_fullres.resize(rep->numCooC);

		for (int i_rep = 0; i_rep < rep->numCooC; i_rep++)
		{
			rep->coocX_fullres[i_rep].resize(rep->sizeCooC[i_rep]);
			rep->coocY_fullres[i_rep].resize(rep->sizeCooC[i_rep]);
			for (int j_rep = 0; j_rep < rep->sizeCooC[i_rep]; j_rep++)
			{
				in>>rep->coocX_fullres[i_rep][j_rep];
				in>>rep->coocY_fullres[i_rep][j_rep];
			}
		}
	}

	// scale the input labels
	rep->coocX_scaled.resize(rep->numCooC);
	rep->coocY_scaled.resize(rep->numCooC);
	for (int i_rep = 0; i_rep < rep->numCooC; i_rep++)
	{
		rep->coocX_scaled[i_rep].resize(rep->sizeCooC[i_rep]);
		rep->coocY_scaled[i_rep].resize(rep->sizeCooC[i_rep]);
		for (int j_rep = 0; j_rep < rep->sizeCooC[i_rep]; j_rep++)
		{
			// scale and make sure the box does not exceed image boundary
			rep->coocX_scaled[i_rep][j_rep] = round(rep->coocX_fullres[i_rep][j_rep] * scalerRes);
			rep->coocY_scaled[i_rep][j_rep] = round(rep->coocY_fullres[i_rep][j_rep] * scalerRes);
		}
	}
}

void ImageViewer::IniCooCBB(){
	//int num_bbstatisticsX;
	//int num_bbstatisticsy;
	//vector<int> list_bbstatisticsX;
	//vector<int> list_bbstatisticsY;
	//double expansion_bbstatisticsX;
	//double expansion_bbstatisticsY;
	QFile fileCooC(filename_coocInput);
	if(fileCooC.open(QIODevice::ReadOnly))
	{
		QTextStream in(&fileCooC);
		in >> rep->num_bbstatisticsX;
		in >> rep->num_bbstatisticsY;
		//qDebug()<<"rep->num_bbstatisticsX: "<<rep->num_bbstatisticsX<<", rep->num_bbstatisticsY: "<<rep->num_bbstatisticsY;
		rep->list_bbstatisticsX.resize(rep->num_bbstatisticsX);
		rep->list_bbstatisticsY.resize(rep->num_bbstatisticsY);
		for (int i_x = 0; i_x < rep->num_bbstatisticsX; i_x++)
		{
			in >>rep->list_bbstatisticsX[i_x];
			//qDebug()<<rep->list_bbstatisticsX[i_x];
		}
		for (int i_y = 0; i_y < rep->num_bbstatisticsY; i_y++)
		{
			in >>rep->list_bbstatisticsY[i_y];
			//qDebug()<<rep->list_bbstatisticsY[i_y];
		}
	}

	//int gen_X = rand() % rep->num_bbstatisticsX;
	//int gen_Y = rand() % rep->num_bbstatisticsY;

	int gen_X = 0;
	int gen_Y = 0;
	if (rep->list_bbstatisticsX.size() > 0)
	{
		rep->expansion_bbstatisticsX = rep->list_bbstatisticsX[gen_X];
	}
	if (rep->list_bbstatisticsY.size() > 0)
	{
		rep->expansion_bbstatisticsY = rep->list_bbstatisticsY[gen_Y];
	}
	
	rep->expansion_bbstatisticsX_scaled = round(rep->expansion_bbstatisticsX * scalerRes);
	rep->expansion_bbstatisticsY_scaled = round(rep->expansion_bbstatisticsY * scalerRes);

	//update expansion_stepX and expansion_stepY
	expansion_stepX = double(rep->expansion_bbstatisticsX_scaled)/double(rep->colsInput_scaled);
	expansion_stepY = double(rep->expansion_bbstatisticsY_scaled)/double(rep->rowsInput_scaled);
	qDebug()<<"********************************************************";
	qDebug()<<"gen_X: "<<gen_X<<", gen_Y: "<<gen_Y;
	qDebug()<<"expansion_stepX: "<<expansion_stepX<<", expansion_stepY: "<<expansion_stepY;
	qDebug()<<"********************************************************";

}

void ImageViewer::open()
{
	synmode = 1; // probably also need to clear a lot of things
	filename_imgInput = QFileDialog::getOpenFileName(this,
		tr("Open File"), directoryname);

	if (!filename_imgInput.isEmpty()) {
		filename_repInput = filename_imgInput.left(filename_imgInput.size() - 4);
		filename_repInput.append("Detection.txt");
		//filename_repInput.append("GT.txt");

		filename_coocInput = filename_imgInput.left(filename_imgInput.size() - 4);
		filename_coocInput.append("BBOccurrenceDetection.txt");
		//filename_coocInput.append("BBOccurrenceGT.txt");
		//filename_coocInput.append("OccurrenceCrossDetection.txt");
		//filename_coocInput.append("OccurrenceCrossGT.txt");

		IniImg(); // Initialize images
		IniRep(); // Initialize repetitive BB
	    IniCooCBB(); // Initial BB coocurrence statistics
		//IniCooC(); // Initialize co-occurrence statistics
		if (flag_constellation)
		{
			computeKNNbb(); // compute constellation model for each pixel
			//computeConsMtHungarian(); // compute constellation distance matrix
			computeConsMDS();
		}

		imgDisp = new QGraphicsPixmapItem(QPixmap::fromImage(*rep->qimgInput_fullres));
		scene->clear();
		scene->addItem(imgDisp);
		scene->setSceneRect(0, 0, rep->qimgInput_fullres->width(), rep->qimgInput_fullres->height());
		view = new QGraphicsView(scene);
		setCentralWidget(view);
		resize(rep->qimgInput_fullres->width() + 10, rep->qimgInput_fullres->height() + 50);
	}
}

void ImageViewer::createActions()
{
	openAct = new QAction(tr("&Open..."), this);
	openAct->setShortcut(tr("Ctrl+P"));
	connect(openAct, SIGNAL(triggered()), this, SLOT(open()));

	saveAct = new QAction(tr("&save"), this);
	saveAct->setShortcut(tr("S"));
	connect(saveAct, SIGNAL(triggered()), this, SLOT(save()));

	printAct = new QAction(tr("&Print..."), this);
	printAct->setShortcut(tr("Ctrl+P"));
	printAct->setEnabled(false);
	connect(printAct, SIGNAL(triggered()), this, SLOT(print()));

	//exitAct = new QAction(tr("E&xit"), this);
	//exitAct->setShortcut(tr("Escape"));
	//connect(exitAct, SIGNAL(triggered()), this, SLOT(close()));

	//zoomInAct = new QAction(tr("Zoom &In (25%)"), this);
	//zoomInAct->setShortcut(tr("Ctrl+I"));
	//zoomInAct->setEnabled(false);
	//connect(zoomInAct, SIGNAL(triggered()), this, SLOT(zoomIn()));

	//zoomOutAct = new QAction(tr("Zoom &Out (25%)"), this);
	//zoomOutAct->setShortcut(tr("Ctrl+O"));
	//zoomOutAct->setEnabled(false);
	//connect(zoomOutAct, SIGNAL(triggered()), this, SLOT(zoomOut()));

	//normalSizeAct = new QAction(tr("&Normal Size"), this);
	//normalSizeAct->setShortcut(tr("Ctrl+N"));
	//normalSizeAct->setEnabled(false);
	//connect(normalSizeAct, SIGNAL(triggered()), this, SLOT(normalSize()));

	//fitToWindowAct = new QAction(tr("&Fit to Window"), this);
	//fitToWindowAct->setEnabled(false);
	//fitToWindowAct->setCheckable(true);
	//fitToWindowAct->setShortcut(tr("Ctrl+F"));
	//connect(fitToWindowAct, SIGNAL(triggered()), this, SLOT(fitToWindow()));

	//aboutAct = new QAction(tr("&About"), this);
	//connect(aboutAct, SIGNAL(triggered()), this, SLOT(about()));

	//aboutQtAct = new QAction(tr("About &Qt"), this);
	//connect(aboutQtAct, SIGNAL(triggered()), qApp, SLOT(aboutQt()));

	//synExpandXAct = new QAction(tr("&ExpandX"), this);
	//synExpandXAct->setShortcut(tr("Ctrl+Right"));
	//connect(synExpandXAct, SIGNAL(triggered()), this, SLOT(synExpandX()));

	//synShrinkXAct = new QAction(tr("&ShrinkX"), this);
	//synShrinkXAct->setShortcut(tr("Ctrl+Left"));
	//connect(synShrinkXAct, SIGNAL(triggered()), this, SLOT(synShrinkX()));

	//synExpandYAct = new QAction(tr("&ExpandY"), this);
	//synExpandYAct->setShortcut(tr("Ctrl+Up"));
	//connect(synExpandYAct, SIGNAL(triggered()), this, SLOT(synExpandY()));

	//synShrinkYAct = new QAction(tr("&ShrinkY"), this);
	//synShrinkYAct->setShortcut(tr("Ctrl+Down"));
	//connect(synShrinkYAct, SIGNAL(triggered()), this, SLOT(synShrinkY()));

	synExpandXAct = new QAction(tr("&ExpandX"), this);
	synExpandXAct->setShortcut(tr("Ctrl+Right"));
	connect(synExpandXAct, SIGNAL(triggered()), this, SLOT(synExpandSampleX()));

	synShrinkXAct = new QAction(tr("&ShrinkX"), this);
	synShrinkXAct->setShortcut(tr("Ctrl+Left"));
	connect(synShrinkXAct, SIGNAL(triggered()), this, SLOT(synShrinkSampleX()));

	synExpandYAct = new QAction(tr("&ExpandY"), this);
	synExpandYAct->setShortcut(tr("Ctrl+Up"));
	connect(synExpandYAct, SIGNAL(triggered()), this, SLOT(synExpandSampleY()));

	synShrinkYAct = new QAction(tr("&ShrinkY"), this);
	synShrinkYAct->setShortcut(tr("Ctrl+Down"));
	connect(synShrinkYAct, SIGNAL(triggered()), this, SLOT(synShrinkSampleY()));

	synOptAct = new QAction(tr("&synOpt"), this);
	synOptAct->setShortcut(tr("Ctrl+O"));
	connect(synOptAct, SIGNAL(triggered()), this, SLOT(synOpt()));

	renderRepLabelAct = new QAction(tr("&renderRepLabel"), this);
	renderRepLabelAct->setShortcut(tr("Ctrl+L"));
	connect(renderRepLabelAct, SIGNAL(triggered()), this, SLOT(renderRepLabel()));

	renderRepBBAct = new QAction(tr("&renderRepBB"), this);
	renderRepBBAct->setShortcut(tr("Ctrl+B"));
	connect(renderRepBBAct, SIGNAL(triggered()), this, SLOT(renderRepBB()));

	scribbleAct = new QAction(tr("&scribble"), this);
	scribbleAct->setShortcut(tr("Ctrl+S"));
	connect(scribbleAct, SIGNAL(triggered()), this, SLOT(scribble()));

	guideStitchAct = new QAction(tr("&guideStitch"), this);
	guideStitchAct->setShortcut(tr("Ctrl+G"));
	connect(guideStitchAct, SIGNAL(triggered()), this, SLOT(synScribble()));

	synSwitchModeAct1 = new QAction(tr("&SynMode1"), this);
	synSwitchModeAct1->setShortcut(tr("Ctrl+1"));
	connect(synSwitchModeAct1, SIGNAL(triggered()), this, SLOT(synSwitchMode1()));


	synSwitchMSLAct = new QAction(tr("&synSwitchMSL"), this);
	synSwitchMSLAct->setShortcut(tr("Ctrl+M"));
	connect(synSwitchMSLAct, SIGNAL(triggered()), this, SLOT(synSwitchMSL()));


	//synSwitchModeAct2 = new QAction(tr("&SynMode2"), this);
	//synSwitchModeAct2->setShortcut(tr("Ctrl+2"));
	//connect(synSwitchModeAct2, SIGNAL(triggered()), this, SLOT(synSwitchMode2()));

	//synSwitchModeAct3 = new QAction(tr("&SynMode3"), this);
	//synSwitchModeAct3->setShortcut(tr("Ctrl+3"));
	//connect(synSwitchModeAct3, SIGNAL(triggered()), this, SLOT(synSwitchMode3()));

	//synSwitchModeAct4 = new QAction(tr("&SynMode4"), this);
	//synSwitchModeAct4->setShortcut(tr("Ctrl+4"));
	//connect(synSwitchModeAct4, SIGNAL(triggered()), this, SLOT(synSwitchMode4()));

	//IncrCostSmoothAct = new QAction(tr("&IncrCostSmooth"), this);
	//IncrCostSmoothAct->setShortcut(tr("S+>"));
	//connect(IncrCostSmoothAct, SIGNAL(triggered()), this, SLOT(IncrCostSmooth()));

	//DecrCostSmoothAct = new QAction(tr("&DecrCostSmooth"), this);
	//DecrCostSmoothAct->setShortcut(tr("S+<"));
	//connect(DecrCostSmoothAct, SIGNAL(triggered()), this, SLOT(DecrCostSmooth()));

	//updateGuideAct = new QAction(tr("&updateGuide"), this);
	//updateGuideAct->setShortcut(tr("Ctrl+U"));
	//connect(updateGuideAct, SIGNAL(triggered()), this, SLOT(updateGuide()));

	//multiSelectionAct = new QAction(tr("&multiselection"), this);
	//multiSelectionAct->setShortcut(tr("Ctrl+M"));
	//connect(multiSelectionAct, SIGNAL(triggered()), this, SLOT(multiselection()));

	//showInputAct = new QAction(tr("&showInput"), this);
	//showInputAct->setShortcut(tr("Ctrl+I"));
	//connect(showInputAct, SIGNAL(triggered()), this, SLOT(showInput()));

	//saveAct = new QAction(tr("&save"), this);
	//saveAct->setShortcut(tr("S"));
	//connect(saveAct, SIGNAL(triggered()), this, SLOT(save()));

	//reconfigAct = new QAction(tr("&reconfig"), this);
	//reconfigAct->setShortcut(tr("Ctrl+C"));
	//connect(reconfigAct, SIGNAL(triggered()), this, SLOT(reconfig()));

	//autoStitchAct = new QAction(tr("&autoStitchA"), this);
	//autoStitchAct->setShortcut(tr("Ctrl+A"));
	//connect(autoStitchAct, SIGNAL(triggered()), this, SLOT(autoStitch()));

	//userPaintAct = new QAction(tr("&userPaint"), this);
	//userPaintAct->setShortcut(tr("Ctrl+D"));
	//connect(userPaintAct, SIGNAL(triggered()), this, SLOT(userPaint()));

	//guideStitchAct = new QAction(tr("&guideStitch"), this);
	//guideStitchAct->setShortcut(tr("Ctrl+G"));
	//connect(guideStitchAct, SIGNAL(triggered()), this, SLOT(guideStitch()));

}

void ImageViewer::createActions2(){
	//guideStitchAct = new QAction(tr("&guideStitch2"), this);
	//guideStitchAct->setShortcut(tr("Ctrl+K"));
	//connect(guideStitchAct, SIGNAL(triggered()), this, SLOT(guideStitch3()));
}

void ImageViewer::createMenus()
{
	fileMenu = new QMenu(tr("&File"), this);
	fileMenu->addAction(openAct);
	fileMenu->addAction(saveAct);
	fileMenu->addAction(printAct);
	//fileMenu->addSeparator();
	//fileMenu->addAction(exitAct);

	//viewMenu = new QMenu(tr("&View"), this);
	//viewMenu->addAction(zoomInAct);
	//viewMenu->addAction(zoomOutAct);
	//viewMenu->addAction(normalSizeAct);
	//viewMenu->addSeparator();
	//viewMenu->addAction(fitToWindowAct);

	helpMenu = new QMenu(tr("&Help"), this);
	helpMenu->addAction(synExpandXAct);
	helpMenu->addAction(synShrinkXAct);
	helpMenu->addAction(synExpandYAct);
	helpMenu->addAction(synShrinkYAct);

	helpMenu->addAction(renderRepLabelAct);
	helpMenu->addAction(synOptAct);
	helpMenu->addAction(renderRepBBAct);
	//helpMenu->addAction(renderBBAct);

	helpMenu->addAction(synSwitchModeAct1);
	//helpMenu->addAction(synSwitchModeAct2);
	//helpMenu->addAction(synSwitchModeAct3);
	//helpMenu->addAction(synSwitchModeAct4);
	//helpMenu->addAction(IncrCostSmoothAct);
	//helpMenu->addAction(DecrCostSmoothAct);
	//
	//
	//helpMenu->addAction(updateGuideAct);
	//helpMenu->addAction(multiSelectionAct);
	//helpMenu->addAction(showInputAct);
	//helpMenu->addAction(reconfigAct);
	//helpMenu->addAction(autoStitchAct);
	//helpMenu->addAction(userPaintAct);
	
	helpMenu->addAction(scribbleAct);
	helpMenu->addAction(guideStitchAct);
	helpMenu->addAction(synSwitchMSLAct);

	menuBar()->addMenu(fileMenu);
	//menuBar()->addMenu(viewMenu);
	menuBar()->addMenu(helpMenu);
}

void ImageViewer::createMenus2(){
	//helpMenuScribble = new QMenu(tr("&helpScribble"), this);
	//helpMenuScribble->addAction(guideStitchAct);
	//menuBar()->addMenu(helpMenuScribble);
}

ImageViewer::ImageViewer(){
	// connect signal from bbitems
	connect(globalsender, SIGNAL(itemmoved()), this, SLOT(updateGuide()));
	connect(globalsender, SIGNAL(bbtypechanged()), this, SLOT(changetypeBB()));

	// initialize color and pen for draw building blocks
	colorList.resize(20);
	colorList[0] = makeVector3f(102.0, 153.0, 255.0);
	colorList[1] = makeVector3f(255.0, 127.0, 102.0);
	colorList[2] = makeVector3f(102.0, 255.0, 127.0);
	colorList[3] = makeVector3f(102.0, 230.0, 255.0);
	colorList[4] = makeVector3f(255.0, 204.0, 102.0);
	colorList[5] = makeVector3f(230.0, 255.0, 102.0);
	colorList[6] = makeVector3f(102.0, 255.0, 204.0);
	colorList[7] = makeVector3f(255.0, 102.0, 153.0);
	colorList[8] = makeVector3f(204.0, 102.0, 255.0);
	colorList[9] = makeVector3f(153.0, 255.0, 102.0);
	colorList[10] = makeVector3f(0.0, 0.0, 0.0);
	colorList[11] = makeVector3f(0.0, 0.0, 0.0);
	colorList[12] = makeVector3f(0.0, 0.0, 0.0);
	colorList[13] = makeVector3f(0.0, 0.0, 0.0);
	colorList[14] = makeVector3f(0.0, 0.0, 0.0);
	colorList[15] = makeVector3f(0.0, 0.0, 0.0);
	colorList[16] = makeVector3f(0.0, 0.0, 0.0);
	colorList[17] = makeVector3f(0.0, 0.0, 0.0);
	colorList[18] = makeVector3f(0.0, 0.0, 0.0);
	colorList[19] = makeVector3f(0.0, 0.0, 0.0);
	pen.setStyle(Qt::SolidLine);
	pen.setWidth(3);

	// initialize data
	synmode = 1; // default synthesis method is one. two and three are for label and label+interlabel. However, these two methods do not work well due to the labels are used in a local way -- pairwise smoothness cost of ADJACENT PIXELS

	IniImg(); // Initialize images
	IniRep(); // Initialize repetitive BB
	IniCooCBB();
	//IniCooC(); // Initialize co-occurrence statistics
	//IniCooCCross(); // Initialize co-occurrence statistics
	if (flag_constellation)
	{
		computeKNNbb(); // compute constellation model for each pixel
		//computeConsMtHungarian(); // compute constellation distance matrix
		computeConsMDS();
	}

	// need to creat a QGraphicsView and a QGraphcisScene
	scene = new QGraphicsScene();
	imgDisp = new QGraphicsPixmapItem(QPixmap::fromImage(*rep->qimgInput_fullres));
	scene->addItem(imgDisp);
	scene->setSceneRect(0, 0, rep->qimgInput_fullres->width(), rep->qimgInput_fullres->height());

	// creat action and menus
	createActions();
	createMenus();
	createActions2();
	createMenus2();

	// render the view
	view = new QGraphicsView(scene);
	setCentralWidget(view);
	resize(rep->qimgInput_fullres->width() + 10, rep->qimgInput_fullres->height() + 50); // due to manual bar the height needs extra space
	setWindowTitle(tr("ImageSyn"));
}

void ImageViewer::synExpandX(){
	synmode = 1;
	scalerSynX += expansion_stepX;
	num_shiftX_scaled += incr_iter_rec;
	synFree();
	//synthesis();
}

void ImageViewer::synExpandY(){
	synmode = 1;
	scalerSynY += expansion_stepY;
	num_shiftY_scaled += incr_iter_rec;
	synFree();
	//synthesisY();
}

void ImageViewer::synShrinkX(){
	synmode = 1;
	if (scalerSynX>1 + expansion_stepX + 0.1)
	{
		scalerSynX -= expansion_stepX;
		num_shiftX_scaled -= incr_iter_rec;
		num_shiftX_scaled = max(1, num_shiftX_scaled);
		synFree();
	}
	else{
		// fake a cheap synthesis
		scalerSynX = 1;
		num_shiftX_scaled = 1;
		if (scalerSynY > 1)
		{
			synFree();
		} 
		else
		{
			// show input image
			imgDisp = new QGraphicsPixmapItem(QPixmap::fromImage(*rep->qimgInput_fullres));
			scene->clear();
			scene->addItem(imgDisp);
			scene->setSceneRect(0, 0, rep->qimgInput_fullres->width(), rep->qimgInput_fullres->height());
			view = new QGraphicsView(scene);
			setCentralWidget(view);
			resize(rep->qimgInput_fullres->width() + 10, rep->qimgInput_fullres->height() + 50);
		}

	}

	//synthesis();
}

void ImageViewer::synShrinkY(){
	synmode = 1;
	if (scalerSynY>1 + expansion_stepY  + 0.1)
	{
		scalerSynY -= expansion_stepY;
		num_shiftY_scaled -= incr_iter_rec;
		num_shiftY_scaled = max(1, num_shiftY_scaled);
		synFree();
	}
	else
	{
		// fake a cheap synthesis
		scalerSynY = 1;
		num_shiftY_scaled = 1;
		//show input image
		if (scalerSynX > 1)
		{
			synFree();
		} 
		else
		{
			imgDisp = new QGraphicsPixmapItem(QPixmap::fromImage(*rep->qimgInput_fullres));
			scene->clear();
			scene->addItem(imgDisp);
			scene->setSceneRect(0, 0, rep->qimgInput_fullres->width(), rep->qimgInput_fullres->height());
			view = new QGraphicsView(scene);
			setCentralWidget(view);
			resize(rep->qimgInput_fullres->width() + 10, rep->qimgInput_fullres->height() + 50);
		}

	}

	//synthesisY();
}

void ImageViewer::synFree(){
	qDebug()<<"synthesis started: ";
	if (synmode != 4)
	{
		// first, compute the dimension of the synthesized image and candidate shifts
		// the following simply regularly samples candidate shift, and no shift reaches out of the synthesized image
		// we can also go cross the image boundary
		// how every with the size of the image grows, the distance between adjacent shifts converges to the multiplication of the step of image expansion/shrinking
		rep->colsSyn_scaled = round(rep->colsInput_scaled * scalerSynX);
		rep->num_shiftX = num_shiftX_scaled;
		rep->dist_shiftX_scaled = round((rep->colsSyn_scaled - rep->colsInput_scaled)/(max(1, rep->num_shiftX - 1)));
		rep->colsSyn_scaled = rep->colsInput_scaled + (rep->num_shiftX - 1) * rep->dist_shiftX_scaled;	

		rep->rowsSyn_scaled = round(rep->rowsInput_scaled * scalerSynY);
		rep->num_shiftY = num_shiftY_scaled;
		rep->dist_shiftY_scaled = round((rep->rowsSyn_scaled - rep->rowsInput_scaled)/(max(1, rep->num_shiftY - 1)));
		rep->rowsSyn_scaled = rep->rowsInput_scaled + (rep->num_shiftY - 1) * rep->dist_shiftY_scaled;	

		inputcols = rep->colsInput_scaled;
		inputrows = rep->rowsInput_scaled;
		syncols = rep->colsSyn_scaled;
		synrows = rep->rowsSyn_scaled;

		rep->numPixelSyn_scaled = rep->rowsSyn_scaled * rep->colsSyn_scaled;
		rep->imgSynGray_scaled = Mat1b::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled); //or, rep->imgSynGray_scaled.create(rows, cols);
		rep->gcolabelSyn_scaled = Mat1b::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled);


		rep->list_shiftX_scaled.resize(rep->num_shiftX);
		for (int i_s = 0; i_s < rep->num_shiftX; i_s++)
		{
			rep->list_shiftX_scaled[i_s] = i_s * rep->dist_shiftX_scaled;
		}
		rep->list_shiftY_scaled.resize(rep->num_shiftY);
		for (int i_s = 0; i_s < rep->num_shiftY; i_s++)
		{
			rep->list_shiftY_scaled[i_s] = i_s * rep->dist_shiftY_scaled;
		}

		// compute the candidate shift
		rep->num_shiftXY = rep->num_shiftX * rep->num_shiftY;
		rep->list_shiftXY_scaled.resize(rep->num_shiftXY);

		vector<Point2i>().swap(gcLabels); // free memory
		for (int i_s = 0; i_s < rep->num_shiftX; i_s++)
		{
			for (int j_s = 0; j_s < rep->num_shiftY; j_s++)
			{
				Point2i* temp = new Point2i( i_s * rep->dist_shiftX_scaled, j_s * rep->dist_shiftY_scaled);
				rep->list_shiftXY_scaled[i_s * rep->num_shiftY + j_s] = *temp;
				gcLabels.push_back(rep->list_shiftXY_scaled[i_s * rep->num_shiftY + j_s]);
			}
		}
	}
	else
	{
		rep->imgSynGray_scaled = Mat1b::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled); //or, rep->imgSynGray_scaled.create(rows, cols);
		rep->gcolabelSyn_scaled = Mat1b::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled);
	}

	qDebug()<<"*************************************";
	qDebug()<<"Candidate shifts for this round: ";
	qDebug()<<"*************************************";
	for (int i_s = 0; i_s < rep->num_shiftXY; i_s++){
		qDebug()<<rep->list_shiftXY_scaled[i_s].x<<rep->list_shiftXY_scaled[i_s].y;
	}
	//for (int i_s = 0; i_s < rep->num_shiftXY; i_s++){
	//	qDebug()<<gcLabels[i_s].x<<gcLabels[i_s].y;
	//}

	vector<Point2i>().swap(gcNodes); // free memory
	gcNodes = point2Node(rep->imgSynGray_scaled);
	//qDebug()<<"size of scaled synthesis image: row: "<<rep->rowsSyn_scaled<<", col: "<<rep->colsSyn_scaled;

	// set up the gco problem and solve it using swap
	GCoptimizationGridGraph *gc = new GCoptimizationGridGraph(rep->colsSyn_scaled, rep->rowsSyn_scaled, rep->num_shiftXY);

	if (synmode == 1)
	{
		gc->setDataCost( &dataFnborder);
	    gc->setSmoothCost( &smoothFn1);
		//gc->setSmoothCost(&SmoothCostFnPotts);
		//gc->setSmoothCost(&smoothFn3);
	}
	else if (synmode == 2)
	{
		//gc->setDataCost( &dataFn);
		//gc->setSmoothCost( &smoothFn2);
	}
	else if (synmode == 3)
	{
		//gc->setDataCost( &dataFn);
		//gc->setSmoothCost(&smoothFn3);
	}
	else if (synmode == 4)
	{
		qDebug()<<"Synthesis mode 4";
		gc->setDataCost( &dataFnborder);
		//gc->setDataCost( &dataFnborderPrior);
		gc->setSmoothCost( &smoothFn3);
	}

	// you can manually set the initial label, this actually influences the result
	//int init_label = num_shiftY_scaled - 1;
	//int init_label = 0;
	//for (int i_l = 0; i_l < rep->numPixelSyn_scaled; i_l++)
	//{
	//	//gc->setLabel(i_l, round(num_shiftY_scaled/2));
	//	gc->setLabel(i_l, init_label);
	//}
	//qDebug()<<"Initial label is "<<init_label;

	qDebug()<<"Before optimization energy is "<<gc->compute_energy();
	//gc->swap(1);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
	gc->expansion(1);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
	qDebug()<<"after expansion energy is "<<gc->compute_energy();
	gc->swap(1);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
	qDebug()<<"after swap energy is "<<gc->compute_energy();
	gc->expansion(1);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
	qDebug()<<"after expansion energy is "<<gc->compute_energy();
	gc->swap(1);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
	qDebug()<<"after swap energy is "<<gc->compute_energy();
	//------------------------------------------------------------------------------
	//// Oh, we need to stitch the candidates by the gc labels
	//------------------------------------------------------------------------------
	//qDebug()<<rep->rowsInput<<rep->colsInput<<endl;
	//qDebug()<<rep->rowsSyn<<rep->colsSyn<<endl;
	//for (int i = 0; i < gcLabels.size(); i++)
	//{
	//	qDebug()<<gcLabels[i].x<<gcLabels[i].y<<endl;
	//}
	rep->imgSyn_scaled = Mat3b::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled);
	qDebug()<<"rep->colsSyn_scaled: "<<rep->colsSyn_scaled<<"rep->rowsSyn_scaled: "<<rep->rowsSyn_scaled;
	qDebug()<<"rep->colsInput_scaled: "<<rep->colsInput_scaled<<"rep->rowsInput_scaled: "<<rep->rowsInput_scaled;
	for ( int  i = 0; i < rep->numPixelSyn_scaled; i++ ){
		int label = gc->whatLabel(i);
		int newX = -gcLabels[label].x + gcNodes[i].x;
		int newY = -gcLabels[label].y + gcNodes[i].y;	

		if (newX >= 0 && newX < rep->colsInput_scaled && newY >= 0 && newY < rep->rowsInput_scaled){
			rep->imgSyn_scaled(gcNodes[i].y, gcNodes[i].x) = rep->imgInput_scaled(newY, newX);
			rep->gcolabelSyn_scaled(gcNodes[i].y, gcNodes[i].x) = label;
		}
	}

	*rep->qimgSyn_scaled = Mat2QImage(rep->imgSyn_scaled);
	rep->qlabelSyn_scaled->setPixmap(QPixmap::fromImage(*rep->qimgSyn_scaled));

	// how about create a fullres now?
	rep->rowsSyn_fullres = rep->rowsInput_fullres;
	rep->dist_shiftX_fullres = rep->dist_shiftX_scaled/scalerRes;
	rep->list_shiftXY_fullres.resize(rep->num_shiftXY);
	for (int i_s = 0; i_s < rep->num_shiftXY; i_s++)
	{
		rep->list_shiftXY_fullres[i_s].x = rep->list_shiftXY_scaled[i_s].x/scalerRes;
		rep->list_shiftXY_fullres[i_s].y = rep->list_shiftXY_scaled[i_s].y/scalerRes;
	}
	rep->rowsSyn_fullres = rep->rowsSyn_scaled/scalerRes;
	rep->colsSyn_fullres = rep->colsSyn_scaled/scalerRes;
	qDebug()<<"size of fullres synthesis image: row: "<<rep->rowsSyn_fullres<<", col: "<<rep->colsSyn_fullres;

	global_rowsSyn_fullres = rep->rowsSyn_fullres;
	global_colsSyn_fullres = rep->colsSyn_fullres;
	global_rowsSyn_scaled = rep->rowsSyn_scaled;
	global_colsSyn_scaled = rep->colsSyn_scaled;

	rep->imgSyn_fullres = Mat3b::zeros(rep->rowsSyn_fullres, rep->colsSyn_fullres);
	rep->gcolabelSyn_fullres = Mat1b::zeros(rep->rowsSyn_fullres, rep->colsSyn_fullres);
	cv::resize(rep->gcolabelSyn_scaled, rep->gcolabelSyn_fullres, Size(rep->colsSyn_fullres,rep->rowsSyn_fullres), 0, 0, INTER_NEAREST);
	rep->imgSyn_fullres = Mat3b::zeros(rep->rowsSyn_fullres, rep->colsSyn_fullres);

	//qDebug()<<"Fullres Syn image size: "<<rep->colsSyn_fullres<<"-by-"<<rep->rowsSyn_fullres;
	for (int r = 0; r < rep->rowsSyn_fullres; r++)
	{
		for (int c = 0; c < rep->colsSyn_fullres; c++)
		{
			//if (synmode == 4)
			//{
			//	qDebug()<<(c - rep->list_shift_fullres[rep->gcolabelSyn_fullres(r, c)]);
			//}
			if ((c - rep->list_shiftXY_fullres[rep->gcolabelSyn_fullres(r, c)].x) < rep->colsInput_fullres && (c - rep->list_shiftXY_fullres[rep->gcolabelSyn_fullres(r, c)].x) >= 0)
			{
				if((r - rep->list_shiftXY_fullres[rep->gcolabelSyn_fullres(r, c)].y) < rep->rowsInput_fullres && (r - rep->list_shiftXY_fullres[rep->gcolabelSyn_fullres(r, c)].y) >= 0)
				{
					rep->imgSyn_fullres(r, c) = rep->imgInput_fullres(r - rep->list_shiftXY_fullres[rep->gcolabelSyn_fullres(r, c)].y, c - rep->list_shiftXY_fullres[rep->gcolabelSyn_fullres(r, c)].x);
				}

			} 
			else
			{
				//qDebug()<<(c - rep->list_shift_fullres[rep->gcolabelSyn_fullres(r, c)]);
			}

		}
	}
	*rep->qimgSyn_fullres = Mat2QImage(rep->imgSyn_fullres);
	rep->qlabelSyn_fullres = new QLabel;
	rep->qlabelSyn_fullres->setPixmap(QPixmap::fromImage(*rep->qimgSyn_fullres));

	// compute the label image for synthesized image 
	rep->imgGuide_scaled = Mat1b::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled);
	for (int r = 0; r < rep->rowsSyn_scaled; r++)
	{
		for (int c = 0; c < rep->colsSyn_scaled; c++)
		{
			if ((c - rep->list_shiftXY_scaled[rep->gcolabelSyn_scaled(r, c)].x) < rep->colsInput_scaled && (c - rep->list_shiftXY_scaled[rep->gcolabelSyn_scaled(r, c)].x) >= 0)
			{
				if((r - rep->list_shiftXY_scaled[rep->gcolabelSyn_scaled(r, c)].y) < rep->rowsInput_scaled && (r - rep->list_shiftXY_scaled[rep->gcolabelSyn_scaled(r, c)].y) >= 0)
				{
					rep->imgGuide_scaled(r, c) = rep->imgInputlabel_scaled(r - rep->list_shiftXY_scaled[rep->gcolabelSyn_scaled(r, c)].y, c - rep->list_shiftXY_scaled[rep->gcolabelSyn_scaled(r, c)].x);
				}
			}
		}
	}
	rep->qimgSynlabelColor_fullres = new QImage(rep->qimgSyn_fullres->size(), QImage::Format_ARGB32);
	for (int r = 0; r < rep->rowsSyn_fullres; r++)
	{
		for (int c = 0; c < rep->colsSyn_fullres; c++)
		{
			if ((c - rep->list_shiftXY_fullres[rep->gcolabelSyn_fullres(r, c)].x) < rep->colsInput_fullres && (c - rep->list_shiftXY_fullres[rep->gcolabelSyn_fullres(r, c)].x) >= 0)
			{
				if((r - rep->list_shiftXY_fullres[rep->gcolabelSyn_fullres(r, c)].y) < rep->rowsInput_fullres && (r - rep->list_shiftXY_fullres[rep->gcolabelSyn_fullres(r, c)].y) >= 0)
				{
					int label = rep->imgInputlabel_fullres(r - rep->list_shiftXY_fullres[rep->gcolabelSyn_fullres(r, c)].y, c - rep->list_shiftXY_fullres[rep->gcolabelSyn_fullres(r, c)].x) - 1;
					if (label > -1)
					{
						QRgb value = qRgb(colorList[label][0], colorList[label][1], colorList[label][2]);
						rep->qimgSynlabelColor_fullres->setPixel(c, r, value);
					}
				}
			} 
			else
			{
				//qDebug()<<(c - rep->list_shift_fullres[rep->gcolabelSyn_fullres(r, c)]);
			}
		}
	}

	rep->qlabelSynlabelColor_fullres = new QLabel;
	rep->qlabelSynlabelColor_fullres->setPixmap(QPixmap::fromImage(*rep->qimgSynlabelColor_fullres));
	//rep->qlabelSynlabelColor_fullres->show();
	imgDisp = new QGraphicsPixmapItem(QPixmap::fromImage(*rep->qimgSyn_fullres));

	if (synmode != 4)
	{
		// render result in the current window
		scene->clear();
		scene->addItem(imgDisp);
		scene->setSceneRect(0, 0, rep->qimgSyn_fullres->width(), rep->qimgSyn_fullres->height());
		view = new QGraphicsView(scene);
		setCentralWidget(view);
		resize(rep->qimgSyn_fullres->width() + 10, rep->qimgSyn_fullres->height() + 50);	
		QString filename_img_syn = "C:/Chuan/Dropbox/Project/2DBuildingBlock/SiggraphAsia/Code/ImageSyn/imageviewer/result/img_syn";
		QString filename_label_syn = "C:/Chuan/Dropbox/Project/2DBuildingBlock/SiggraphAsia/Code/ImageSyn/imageviewer/result/label_syn.jpg";
		filename_img_syn.append("_");
		filename_img_syn.append(QString ("%1").arg(0));
		filename_img_syn.append(".jpg");
		filename_label_syn.append("_");
		filename_label_syn.append(QString ("%1").arg(0));
		filename_label_syn.append(".jpg");
		rep->qimgSyn_fullres->save(filename_img_syn);
		rep->qimgSynlabelColor_fullres->save(filename_label_syn);
	} 
	else
	{
		//save intermediate results
		// we need to save 
		// rep->qimgSyn_fullres
		// rep->qimgSynlabelColor_fullres
		scene->clear();
		scene->addItem(imgDisp);
		scene->setSceneRect(0, 0, rep->qimgSyn_fullres->width(), rep->qimgSyn_fullres->height());
		view = new QGraphicsView(scene);
		setCentralWidget(view);
		resize(rep->qimgSyn_fullres->width() + 10, rep->qimgSyn_fullres->height() + 50);	
		QString filename_img_syn = "C:/Chuan/Dropbox/Project/2DBuildingBlock/SiggraphAsia/Code/ImageSyn/imageviewer/result/img_syn";
		QString filename_label_syn = "C:/Chuan/Dropbox/Project/2DBuildingBlock/SiggraphAsia/Code/ImageSyn/imageviewer/result/label_syn.jpg";
		filename_img_syn.append("_");
		filename_img_syn.append(QString ("%1").arg(i_iter_em + 1));
		filename_img_syn.append(".jpg");
		filename_label_syn.append("_");
		filename_label_syn.append(QString ("%1").arg(i_iter_em + 1));
		filename_label_syn.append(".jpg");
		rep->qimgSyn_fullres->save(filename_img_syn);
		rep->qimgSynlabelColor_fullres->save(filename_label_syn);
	}

	qDebug()<<"synthesis finished: ";
}

void ImageViewer::synExpandSampleX(){
	synmode = default_syn_mode;
	flag_interactive = false;
	flag_scribble = false;
	// sample expansion_stepX from rep->list_bbstatisticsX
	int gen_X = rand() % rep->num_bbstatisticsX;
	//int gen_X = 1;
	rep->expansion_bbstatisticsX = rep->list_bbstatisticsX[gen_X];
	rep->expansion_bbstatisticsX_scaled = round(rep->expansion_bbstatisticsX * scalerRes);
	//rep->expansion_bbstatisticsX_scaled = 15; 
	// increase num_shiftX_scaled by incr_iter_rec, should fix to one really
	num_shiftX_scaled += incr_iter_rec;

	// update 		
	rep->colsSyn_scaled += rep->expansion_bbstatisticsX_scaled;
	rep->num_shiftX += incr_iter_rec;
	// add new shift to rep->list_shiftX_scaled
    rep->list_shiftX_scaled.push_back(rep->list_shiftX_scaled[rep->list_shiftX_scaled.size() - 1] + rep->expansion_bbstatisticsX_scaled);
	qDebug()<<"**************************************************************************************";
	qDebug()<<"The sampled rep->expansion_bbstatisticsX_scaled: "<<rep->expansion_bbstatisticsX_scaled;
	qDebug()<<"**************************************************************************************";
	if (flag_MS)
	{
		synSampleMSL();
	} 
	else
	{
		synSample();
	}

}

void ImageViewer::synShrinkSampleX(){
	synmode = default_syn_mode;
	flag_interactive = false;
	flag_scribble = false;
	if (rep->num_shiftX> 2)
	{
		rep->num_shiftX -= incr_iter_rec;
		num_shiftX_scaled -= incr_iter_rec;
		rep->colsSyn_scaled -= rep->list_shiftX_scaled[rep->list_shiftX_scaled.size() - 1] - rep->list_shiftX_scaled[rep->list_shiftX_scaled.size() - 2];
		rep->list_shiftX_scaled.pop_back();
		if (flag_MS)
		{
			synSampleMSL();
		} 
		else
		{
			synSample();
		}

	}
	else{
		if (rep->num_shiftY > 1)
		{
			if (rep->num_shiftX == 2)
			{
				rep->num_shiftX -= incr_iter_rec;
				num_shiftX_scaled -= incr_iter_rec;
				rep->colsSyn_scaled = rep->colsInput_scaled;
				rep->list_shiftX_scaled.pop_back();
			}
			if (flag_MS)
			{
				synSampleMSL();
			} 
			else
			{
				synSample();
			}

		} 
		else
		{
			rep->num_shiftX = 1;
			rep->num_shiftY = 1;
			num_shiftX_scaled = 1;
			num_shiftY_scaled = 1;
			rep->colsSyn_scaled = rep->colsInput_scaled;
			rep->rowsSyn_scaled = rep->rowsInput_scaled;
			vector<int>().swap(rep->list_shiftX_scaled);
			vector<int>().swap(rep->list_shiftY_scaled);
			rep->list_shiftX_scaled.resize(1);
			rep->list_shiftY_scaled.resize(1);
			rep->list_shiftX_scaled[0] = 0;
			rep->list_shiftY_scaled[0] = 0;
			// show input image
			imgDisp = new QGraphicsPixmapItem(QPixmap::fromImage(*rep->qimgInput_fullres));
			scene->clear();
			scene->addItem(imgDisp);
			scene->setSceneRect(0, 0, rep->qimgInput_fullres->width(), rep->qimgInput_fullres->height());
			view = new QGraphicsView(scene);
			setCentralWidget(view);
			resize(rep->qimgInput_fullres->width() + 10, rep->qimgInput_fullres->height() + 50);
		}
	}
}

void ImageViewer::synExpandSampleY(){
	synmode = default_syn_mode;
	flag_interactive = false;
	flag_scribble = false;
	// sample expansion_stepX from rep->list_bbstatisticsX
	int gen_Y = rand() % rep->num_bbstatisticsY;
	//int gen_Y = 1;
	rep->expansion_bbstatisticsY = rep->list_bbstatisticsY[gen_Y];
	rep->expansion_bbstatisticsY_scaled = round(rep->expansion_bbstatisticsY * scalerRes);
	//rep->expansion_bbstatisticsY_scaled = 8;
	//rep->expansion_bbstatisticsY_scaled = 8; // for teaser vertical

	// increase num_shiftX_scaled by incr_iter_rec, should fix to one really
	num_shiftY_scaled += incr_iter_rec;

	// update 		
	rep->rowsSyn_scaled += rep->expansion_bbstatisticsY_scaled;
	rep->num_shiftY += incr_iter_rec;
	// add new shift to rep->list_shiftX_scaled
	rep->list_shiftY_scaled.push_back(rep->list_shiftY_scaled[rep->list_shiftY_scaled.size() - 1] + rep->expansion_bbstatisticsY_scaled);
	qDebug()<<"**************************************************************************************";
	qDebug()<<"The sampled rep->expansion_bbstatisticsY_scaled: "<<rep->expansion_bbstatisticsY_scaled;
	qDebug()<<"**************************************************************************************";

	if (flag_MS)
	{
		synSampleMSL();
	} 
	else
	{
		synSample();
	}

}

void ImageViewer::synShrinkSampleY(){
	synmode = default_syn_mode;
	flag_interactive = false;
	flag_scribble = false;
	if (rep->num_shiftY > 2)
	{
		rep->num_shiftY -= incr_iter_rec;
		num_shiftY_scaled -= incr_iter_rec;
		rep->rowsSyn_scaled -= rep->list_shiftY_scaled[rep->list_shiftY_scaled.size() - 1] - rep->list_shiftY_scaled[rep->list_shiftY_scaled.size() - 2];
		rep->list_shiftY_scaled.pop_back();
		if (flag_MS)
		{
			synSampleMSL();
		} 
		else
		{
			synSample();
		}

	}
	else{
		if (rep->num_shiftX > 1)
		{
			if (rep->num_shiftY == 2)
			{
				rep->num_shiftY -= incr_iter_rec;
				num_shiftY_scaled -= incr_iter_rec;
				rep->rowsSyn_scaled = rep->rowsInput_scaled;
				rep->list_shiftY_scaled.pop_back();
			}
			if (flag_MS)
			{
				synSampleMSL();
			} 
			else
			{
				synSample();
			}

		} 
		else
		{
			rep->num_shiftX = 1;
			rep->num_shiftY = 1;
			num_shiftX_scaled = 1;
			num_shiftY_scaled = 1;
			rep->colsSyn_scaled = rep->colsInput_scaled;
			rep->rowsSyn_scaled = rep->rowsInput_scaled;
			vector<int>().swap(rep->list_shiftX_scaled);
			vector<int>().swap(rep->list_shiftY_scaled);
			rep->list_shiftX_scaled.resize(1);
			rep->list_shiftY_scaled.resize(1);
			rep->list_shiftX_scaled[0] = 0;
			rep->list_shiftY_scaled[0] = 0;
			// show input image
			imgDisp = new QGraphicsPixmapItem(QPixmap::fromImage(*rep->qimgInput_fullres));
			scene->clear();
			scene->addItem(imgDisp);
			scene->setSceneRect(0, 0, rep->qimgInput_fullres->width(), rep->qimgInput_fullres->height());
			view = new QGraphicsView(scene);
			setCentralWidget(view);
			resize(rep->qimgInput_fullres->width() + 10, rep->qimgInput_fullres->height() + 50);
		}
	}
}

void ImageViewer::synSample(){
	qDebug()<<"To synthesis using statistical sampling";

	inputcols = rep->colsInput_scaled;
	inputrows = rep->rowsInput_scaled;
	syncols = rep->colsSyn_scaled;
	synrows = rep->rowsSyn_scaled;

	rep->numPixelSyn_scaled = rep->rowsSyn_scaled * rep->colsSyn_scaled;
	rep->imgSynGray_scaled = Mat1b::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled); //or, rep->imgSynGray_scaled.create(rows, cols);
	rep->gcolabelSyn_scaled = Mat1b::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled);

	// compute the candidate shift
	rep->num_shiftXY = rep->num_shiftX * rep->num_shiftY;
	rep->list_shiftXY_scaled.resize(rep->num_shiftXY);

	vector<Point2i>().swap(gcLabels); // free memory
	for (int i_s = 0; i_s < rep->num_shiftX; i_s++)
	{
		for (int j_s = 0; j_s < rep->num_shiftY; j_s++)
		{
			Point2i* temp = new Point2i( rep->list_shiftX_scaled[i_s], rep->list_shiftY_scaled[j_s]);
			//Point2i* temp = new Point2i( i_s * rep->dist_shiftX_scaled, j_s * rep->dist_shiftY_scaled);
			rep->list_shiftXY_scaled[i_s * rep->num_shiftY + j_s] = *temp;
			gcLabels.push_back(rep->list_shiftXY_scaled[i_s * rep->num_shiftY + j_s]);
		}
	}


	qDebug()<<"*************************************";
	qDebug()<<"Candidate shifts for this round: ";
	qDebug()<<"*************************************";
	for (int i_s = 0; i_s < rep->num_shiftXY; i_s++){
		qDebug()<<rep->list_shiftXY_scaled[i_s].x<<rep->list_shiftXY_scaled[i_s].y;
	}
	//for (int i_s = 0; i_s < rep->num_shiftXY; i_s++){
	//	qDebug()<<gcLabels[i_s].x<<gcLabels[i_s].y;
	//}

	vector<Point2i>().swap(gcNodes); // free memory
	gcNodes = point2Node(rep->imgSynGray_scaled);
	//qDebug()<<"size of scaled synthesis image: row: "<<rep->rowsSyn_scaled<<", col: "<<rep->colsSyn_scaled;

	// set up the gco problem and solve it using swap
	GCoptimizationGridGraph *gc = new GCoptimizationGridGraph(rep->colsSyn_scaled, rep->rowsSyn_scaled, rep->num_shiftXY);

	if (synmode == 1)
	{
		qDebug()<<"Synthesis mode 1: image resizing";
		qDebug()<<"DataCost term: dataFnborder";
		gc->setDataCost( &dataFnborder);
		qDebug()<<"smoothness term: smoothFn1";
		gc->setSmoothCost( &smoothFn1);
	}
	else if (synmode == 2)
	{
		qDebug()<<"Synthesis mode 2: for interactive editing";
		qDebug()<<"DataCost term: dataFnborderGuide";
		gc->setDataCost( &dataFnborderGuide);
		qDebug()<<"smoothness term: smoothFn1";
		gc->setSmoothCost( &smoothFn1);
	}
	else if (synmode == 3)
	{
		qDebug()<<"Synthesis mode 3: for scribbling";
		qDebug()<<"DataCost term: dataFnborderScribble";
		gc->setDataCost( &dataFnborderScribble);
		qDebug()<<"smoothness term: smoothFn1";
		gc->setSmoothCost( &smoothFn1);
	}
	else if (synmode == 4)
	{
		qDebug()<<"Synthesis mode 4: switch to bb mode.";
		if (!flag_scribble && !flag_interactive)
		{
			qDebug()<<"DataCost term: dataFnborder";
			gc->setDataCost( &dataFnborder);
		}
		
		if (flag_scribble)
		{
			qDebug()<<"DataCost term: dataFnborderScribble";
			gc->setDataCost( &dataFnborderScribble);
		}

		if (flag_interactive)
		{
			qDebug()<<"DataCost term: dataFnborderGuide";
			gc->setDataCost( &dataFnborderGuide);
		}
		if (flag_constellation)
		{
			qDebug()<<"smoothness term: smoothFnConstellation3";
			gc->setSmoothCost(&smoothFnConstellation3);
		} 
		else
		{
			qDebug()<<"smoothness term: smoothFn3";
			gc->setSmoothCost( &smoothFn3);
		}
	}

	for (int i_l = 0; i_l < rep->numPixelSyn_scaled; i_l++)
	{
		gc->setLabel(i_l, ini_label);
	}

	qDebug()<<"Before optimization energy is "<<gc->compute_energy();

    // the not so correct way
	//gc->swap(1);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);


	//// the correct way ...
	gc->expansion(1);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
	qDebug()<<"after expansion energy is "<<gc->compute_energy();
	gc->swap(1);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
	qDebug()<<"after swap energy is "<<gc->compute_energy();
	gc->expansion(1);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
	qDebug()<<"after expansion energy is "<<gc->compute_energy();
	gc->swap(1);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
	qDebug()<<"after swap energy is "<<gc->compute_energy();


	//------------------------------------------------------------------------------
	//// Oh, we need to stitch the candidates by the gc labels
	//------------------------------------------------------------------------------
	rep->imgSyn_scaled = Mat3b::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled);
	qDebug()<<"rep->colsSyn_scaled: "<<rep->colsSyn_scaled<<"rep->rowsSyn_scaled: "<<rep->rowsSyn_scaled;
	qDebug()<<"rep->colsInput_scaled: "<<rep->colsInput_scaled<<"rep->rowsInput_scaled: "<<rep->rowsInput_scaled;
	for ( int  i = 0; i < rep->numPixelSyn_scaled; i++ ){
		int label = gc->whatLabel(i);
		int newX = -gcLabels[label].x + gcNodes[i].x;
		int newY = -gcLabels[label].y + gcNodes[i].y;	

		if (newX >= 0 && newX < rep->colsInput_scaled && newY >= 0 && newY < rep->rowsInput_scaled){
			rep->imgSyn_scaled(gcNodes[i].y, gcNodes[i].x) = rep->imgInput_scaled(newY, newX);
			rep->gcolabelSyn_scaled(gcNodes[i].y, gcNodes[i].x) = label;
		}
	}

	*rep->qimgSyn_scaled = Mat2QImage(rep->imgSyn_scaled);
	rep->qlabelSyn_scaled->setPixmap(QPixmap::fromImage(*rep->qimgSyn_scaled));


	// how about create a fullres now?
	rep->rowsSyn_fullres = rep->rowsInput_fullres;
	rep->list_shiftXY_fullres.resize(rep->num_shiftXY);
	for (int i_s = 0; i_s < rep->num_shiftXY; i_s++)
	{
		rep->list_shiftXY_fullres[i_s].x = rep->list_shiftXY_scaled[i_s].x/scalerRes;
		rep->list_shiftXY_fullres[i_s].y = rep->list_shiftXY_scaled[i_s].y/scalerRes;
	}
	rep->rowsSyn_fullres = rep->rowsSyn_scaled/scalerRes;
	rep->colsSyn_fullres = rep->colsSyn_scaled/scalerRes;
	qDebug()<<"size of fullres synthesis image: row: "<<rep->rowsSyn_fullres<<", col: "<<rep->colsSyn_fullres;

	global_rowsSyn_fullres = rep->rowsSyn_fullres;
	global_colsSyn_fullres = rep->colsSyn_fullres;
	global_rowsSyn_scaled = rep->rowsSyn_scaled;
	global_colsSyn_scaled = rep->colsSyn_scaled;

	rep->imgSyn_fullres = Mat3b::zeros(rep->rowsSyn_fullres, rep->colsSyn_fullres);
	rep->gcolabelSyn_fullres = Mat1b::zeros(rep->rowsSyn_fullres, rep->colsSyn_fullres);
	cv::resize(rep->gcolabelSyn_scaled, rep->gcolabelSyn_fullres, Size(rep->colsSyn_fullres,rep->rowsSyn_fullres), 0, 0, INTER_NEAREST);
	rep->imgSyn_fullres = Mat3b::zeros(rep->rowsSyn_fullres, rep->colsSyn_fullres);

	//qDebug()<<"Fullres Syn image size: "<<rep->colsSyn_fullres<<"-by-"<<rep->rowsSyn_fullres;
	for (int r = 0; r < rep->rowsSyn_fullres; r++)
	{
		for (int c = 0; c < rep->colsSyn_fullres; c++)
		{
			//if (synmode == 4)
			//{
			//	qDebug()<<(c - rep->list_shift_fullres[rep->gcolabelSyn_fullres(r, c)]);
			//}
			if ((c - rep->list_shiftXY_fullres[rep->gcolabelSyn_fullres(r, c)].x) < rep->colsInput_fullres && (c - rep->list_shiftXY_fullres[rep->gcolabelSyn_fullres(r, c)].x) >= 0)
			{
				if((r - rep->list_shiftXY_fullres[rep->gcolabelSyn_fullres(r, c)].y) < rep->rowsInput_fullres && (r - rep->list_shiftXY_fullres[rep->gcolabelSyn_fullres(r, c)].y) >= 0)
				{
					rep->imgSyn_fullres(r, c) = rep->imgInput_fullres(r - rep->list_shiftXY_fullres[rep->gcolabelSyn_fullres(r, c)].y, c - rep->list_shiftXY_fullres[rep->gcolabelSyn_fullres(r, c)].x);
				}

			} 
			else
			{
				//qDebug()<<(c - rep->list_shift_fullres[rep->gcolabelSyn_fullres(r, c)]);
			}

		}
	}
	*rep->qimgSyn_fullres = Mat2QImage(rep->imgSyn_fullres);
	rep->qlabelSyn_fullres = new QLabel;
	rep->qlabelSyn_fullres->setPixmap(QPixmap::fromImage(*rep->qimgSyn_fullres));

	// compute the label image for synthesized image 
	rep->imgGuide_scaled = Mat1b::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled);
	for (int r = 0; r < rep->rowsSyn_scaled; r++)
	{
		for (int c = 0; c < rep->colsSyn_scaled; c++)
		{
			if ((c - rep->list_shiftXY_scaled[rep->gcolabelSyn_scaled(r, c)].x) < rep->colsInput_scaled && (c - rep->list_shiftXY_scaled[rep->gcolabelSyn_scaled(r, c)].x) >= 0)
			{
				if((r - rep->list_shiftXY_scaled[rep->gcolabelSyn_scaled(r, c)].y) < rep->rowsInput_scaled && (r - rep->list_shiftXY_scaled[rep->gcolabelSyn_scaled(r, c)].y) >= 0)
				{
					rep->imgGuide_scaled(r, c) = rep->imgInputlabel_scaled(r - rep->list_shiftXY_scaled[rep->gcolabelSyn_scaled(r, c)].y, c - rep->list_shiftXY_scaled[rep->gcolabelSyn_scaled(r, c)].x);
				}
			}
		}
	}
	rep->qimgSynlabelColor_fullres = new QImage(rep->qimgSyn_fullres->size(), QImage::Format_ARGB32);
	for (int r = 0; r < rep->rowsSyn_fullres; r++)
	{
		for (int c = 0; c < rep->colsSyn_fullres; c++)
		{
			if ((c - rep->list_shiftXY_fullres[rep->gcolabelSyn_fullres(r, c)].x) < rep->colsInput_fullres && (c - rep->list_shiftXY_fullres[rep->gcolabelSyn_fullres(r, c)].x) >= 0)
			{
				if((r - rep->list_shiftXY_fullres[rep->gcolabelSyn_fullres(r, c)].y) < rep->rowsInput_fullres && (r - rep->list_shiftXY_fullres[rep->gcolabelSyn_fullres(r, c)].y) >= 0)
				{
					int label = rep->imgInputlabel_fullres(r - rep->list_shiftXY_fullres[rep->gcolabelSyn_fullres(r, c)].y, c - rep->list_shiftXY_fullres[rep->gcolabelSyn_fullres(r, c)].x) - 1;
					if (label > -1)
					{
						QRgb value = qRgb(colorList[label][0], colorList[label][1], colorList[label][2]);
						rep->qimgSynlabelColor_fullres->setPixel(c, r, value);
					}
				}
			} 
			else
			{
				//qDebug()<<(c - rep->list_shift_fullres[rep->gcolabelSyn_fullres(r, c)]);
			}
		}
	}

	rep->qlabelSynlabelColor_fullres = new QLabel;
	rep->qlabelSynlabelColor_fullres->setPixmap(QPixmap::fromImage(*rep->qimgSynlabelColor_fullres));
	//rep->qlabelSynlabelColor_fullres->show();
	imgDisp = new QGraphicsPixmapItem(QPixmap::fromImage(*rep->qimgSyn_fullres));

	if (synmode != 4)
	{
		// render result in the current window
		scene->clear();
		scene->addItem(imgDisp);
		scene->setSceneRect(0, 0, rep->qimgSyn_fullres->width(), rep->qimgSyn_fullres->height());
		view = new QGraphicsView(scene);
		setCentralWidget(view);
		resize(rep->qimgSyn_fullres->width() + 10, rep->qimgSyn_fullres->height() + 50);	
		QString filename_img_syn = "C:/Chuan/Dropbox/Project/2DBuildingBlock/SiggraphAsia/Code/ImageSyn/imageviewer/result/img_syn";
		QString filename_label_syn = "C:/Chuan/Dropbox/Project/2DBuildingBlock/SiggraphAsia/Code/ImageSyn/imageviewer/result/label_syn.jpg";
		filename_img_syn.append("_");
		filename_img_syn.append(QString ("%1").arg(0));
		filename_img_syn.append(".jpg");
		filename_label_syn.append("_");
		filename_label_syn.append(QString ("%1").arg(0));
		filename_label_syn.append(".jpg");
		rep->qimgSyn_fullres->save(filename_img_syn);
		rep->qimgSynlabelColor_fullres->save(filename_label_syn);
	} 
	else
	{
		//save intermediate results
		// we need to save 
		// rep->qimgSyn_fullres
		// rep->qimgSynlabelColor_fullres
		scene->clear();
		scene->addItem(imgDisp);
		scene->setSceneRect(0, 0, rep->qimgSyn_fullres->width(), rep->qimgSyn_fullres->height());
		view = new QGraphicsView(scene);
		setCentralWidget(view);
		resize(rep->qimgSyn_fullres->width() + 10, rep->qimgSyn_fullres->height() + 50);	
		QString filename_img_syn = "C:/Chuan/Dropbox/Project/2DBuildingBlock/SiggraphAsia/Code/ImageSyn/imageviewer/result/img_syn";
		QString filename_label_syn = "C:/Chuan/Dropbox/Project/2DBuildingBlock/SiggraphAsia/Code/ImageSyn/imageviewer/result/label_syn.jpg";
		filename_img_syn.append("_");
		filename_img_syn.append(QString ("%1").arg(i_iter_em + 1));
		filename_img_syn.append(".jpg");
		filename_label_syn.append("_");
		filename_label_syn.append(QString ("%1").arg(i_iter_em + 1));
		filename_label_syn.append(".jpg");
		rep->qimgSyn_fullres->save(filename_img_syn);
		rep->qimgSynlabelColor_fullres->save(filename_label_syn);
	}

	qDebug()<<"synthesis finished: ";

}

void ImageViewer::synSampleMS(){

	qDebug()<<"Synthesis using statistical sampling and Multi-resolution ";

	inputcols = rep->colsInput_scaled;
	inputrows = rep->rowsInput_scaled;
	syncols = rep->colsSyn_scaled;
	synrows = rep->rowsSyn_scaled;
	inputcols_fullres = rep->colsInput_fullres;
	inputrows_fullres = rep->rowsInput_fullres;

	rep->rowsSyn_fullres = rep->rowsSyn_scaled/scalerRes;
	rep->colsSyn_fullres = rep->colsSyn_scaled/scalerRes;
	syncols_fullres = rep->colsSyn_fullres;
	synrows_fullres = rep->rowsSyn_fullres;
	rep->numPixelSyn_fullres = syncols_fullres * synrows_fullres;

	rep->numPixelSyn_scaled = rep->rowsSyn_scaled * rep->colsSyn_scaled;
	rep->imgSynGray_scaled = Mat1b::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled); //or, rep->imgSynGray_scaled.create(rows, cols);
	rep->imgSynGray_fullres = Mat1b::zeros(rep->rowsSyn_fullres, rep->colsSyn_fullres);
	rep->gcolabelSyn_scaled = Mat1b::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled);

	// compute the candidate shift
	rep->num_shiftXY = rep->num_shiftX * rep->num_shiftY;
	rep->list_shiftXY_scaled.resize(rep->num_shiftXY);
	vector<Point2i>().swap(gcLabels); // free memory
	for (int i_s = 0; i_s < rep->num_shiftX; i_s++)
	{
		for (int j_s = 0; j_s < rep->num_shiftY; j_s++)
		{
			Point2i* temp = new Point2i( rep->list_shiftX_scaled[i_s], rep->list_shiftY_scaled[j_s]);
			rep->list_shiftXY_scaled[i_s * rep->num_shiftY + j_s] = *temp;
			gcLabels.push_back(rep->list_shiftXY_scaled[i_s * rep->num_shiftY + j_s]);
		}
	}

	rep->list_shiftXY_fullres.resize(rep->num_shiftXY);
	vector<Point2i>().swap(gcLabels_fullres); // free memory
	for (int i_s = 0; i_s < rep->num_shiftXY; i_s++)
	{
		rep->list_shiftXY_fullres[i_s].x = rep->list_shiftXY_scaled[i_s].x/scalerRes;
		rep->list_shiftXY_fullres[i_s].y = rep->list_shiftXY_scaled[i_s].y/scalerRes;
	}
	for (int i_s = 0; i_s < rep->num_shiftX; i_s++)
	{
		for (int j_s = 0; j_s < rep->num_shiftY; j_s++)
		{
			gcLabels_fullres.push_back(rep->list_shiftXY_fullres[i_s * rep->num_shiftY + j_s]);
		}
	}


	vector<Point2i>().swap(gcNodes); // free memory
	gcNodes = point2Node(rep->imgSynGray_scaled);
	vector<Point2i>().swap(gcNodes_fullres); // free memory
	gcNodes_fullres = point2Node(rep->imgSynGray_fullres);

	qDebug()<<"size of gcNodes_fullres: "<<gcNodes_fullres.size();

	qDebug()<<"*************************************";
	qDebug()<<"Candidate shifts for this round: ";
	qDebug()<<"*************************************";
	for (int i_s = 0; i_s < rep->num_shiftXY; i_s++){
		qDebug()<<rep->list_shiftXY_scaled[i_s].x<<rep->list_shiftXY_scaled[i_s].y;
	}

	//qDebug()<<"size of scaled synthesis image: row: "<<rep->rowsSyn_scaled<<", col: "<<rep->colsSyn_scaled;

	// set up the gco problem and solve it using swap
	GCoptimizationGridGraph *gc = new GCoptimizationGridGraph(rep->colsSyn_scaled, rep->rowsSyn_scaled, rep->num_shiftXY);

	if (synmode == 1)
	{
		qDebug()<<"Synthesis mode 1: image resizing";
		qDebug()<<"DataCost term: dataFnborder";
		gc->setDataCost( &dataFnborder);
		qDebug()<<"smoothness term: smoothFn1";
		gc->setSmoothCost( &smoothFn1);
	}
	else if (synmode == 2)
	{
		qDebug()<<"Synthesis mode 2: for interactive editing";
		qDebug()<<"DataCost term: dataFnborderGuide";
		gc->setDataCost( &dataFnborderGuide);
		qDebug()<<"smoothness term: smoothFn1";
		gc->setSmoothCost( &smoothFn1);
	}
	else if (synmode == 3)
	{
		qDebug()<<"Synthesis mode 3: for scribbling";
		qDebug()<<"DataCost term: dataFnborderScribble";
		gc->setDataCost( &dataFnborderScribble);
		qDebug()<<"smoothness term: smoothFn1";
		gc->setSmoothCost( &smoothFn1);
	}
	else if (synmode == 4)
	{
		qDebug()<<"Synthesis mode 4: switch to bb mode.";
		if (!flag_scribble && !flag_interactive)
		{
			qDebug()<<"DataCost term: dataFnborder";
			gc->setDataCost( &dataFnborder);
		}

		if (flag_scribble)
		{
			qDebug()<<"DataCost term: dataFnborderScribble";
			gc->setDataCost( &dataFnborderScribble);
		}

		if (flag_interactive)
		{
			qDebug()<<"DataCost term: dataFnborderGuide";
			gc->setDataCost( &dataFnborderGuide);
		}
		if (flag_constellation)
		{
			qDebug()<<"smoothness term: smoothFnConstellation3";
			gc->setSmoothCost(&smoothFnConstellation3);
		} 
		else
		{
			qDebug()<<"smoothness term: smoothFn3";
			gc->setSmoothCost( &smoothFn3);
		}
	}

	for (int i_l = 0; i_l < rep->numPixelSyn_scaled; i_l++)
	{
		gc->setLabel(i_l, ini_label);
	}

	qDebug()<<"Before optimization energy is "<<gc->compute_energy();

	//// the correct way ...
	gc->expansion(1);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
	qDebug()<<"after expansion energy is "<<gc->compute_energy();
	gc->swap(1);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
	qDebug()<<"after swap energy is "<<gc->compute_energy();
	gc->expansion(1);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
	qDebug()<<"after expansion energy is "<<gc->compute_energy();
	gc->swap(1);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
	qDebug()<<"after swap energy is "<<gc->compute_energy();


	//------------------------------------------------------------------------------
	// Stitch the candidates by gc labels
	//------------------------------------------------------------------------------
	rep->imgSyn_scaled = Mat3b::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled);
	qDebug()<<"rep->colsSyn_scaled: "<<rep->colsSyn_scaled<<"rep->rowsSyn_scaled: "<<rep->rowsSyn_scaled;
	qDebug()<<"rep->colsInput_scaled: "<<rep->colsInput_scaled<<"rep->rowsInput_scaled: "<<rep->rowsInput_scaled;
	for ( int  i = 0; i < rep->numPixelSyn_scaled; i++ ){
		int label = gc->whatLabel(i);
		int newX = -gcLabels[label].x + gcNodes[i].x;
		int newY = -gcLabels[label].y + gcNodes[i].y;	

		if (newX >= 0 && newX < rep->colsInput_scaled && newY >= 0 && newY < rep->rowsInput_scaled){
			rep->imgSyn_scaled(gcNodes[i].y, gcNodes[i].x) = rep->imgInput_scaled(newY, newX);
			rep->gcolabelSyn_scaled(gcNodes[i].y, gcNodes[i].x) = label;
		}
	}

	*rep->qimgSyn_scaled = Mat2QImage(rep->imgSyn_scaled);
	rep->qlabelSyn_scaled->setPixmap(QPixmap::fromImage(*rep->qimgSyn_scaled));
	delete gc;

	// compute the label image for synthesized image 
	rep->imgGuide_scaled = Mat1b::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled);
	for (int r = 0; r < rep->rowsSyn_scaled; r++)
	{
		for (int c = 0; c < rep->colsSyn_scaled; c++)
		{
			if ((c - rep->list_shiftXY_scaled[rep->gcolabelSyn_scaled(r, c)].x) < rep->colsInput_scaled && (c - rep->list_shiftXY_scaled[rep->gcolabelSyn_scaled(r, c)].x) >= 0)
			{
				if((r - rep->list_shiftXY_scaled[rep->gcolabelSyn_scaled(r, c)].y) < rep->rowsInput_scaled && (r - rep->list_shiftXY_scaled[rep->gcolabelSyn_scaled(r, c)].y) >= 0)
				{
					rep->imgGuide_scaled(r, c) = rep->imgInputlabel_scaled(r - rep->list_shiftXY_scaled[rep->gcolabelSyn_scaled(r, c)].y, c - rep->list_shiftXY_scaled[rep->gcolabelSyn_scaled(r, c)].x);
				}
			}
		}
	}

	//------------------------------------------------------------------------------
	// scale gcolabelSyn_scaled to gcolabelSyn_fullres
	//------------------------------------------------------------------------------
	cv::resize(rep->gcolabelSyn_scaled, rep->gcolabelSyn_fullres, Size(rep->colsSyn_fullres,rep->rowsSyn_fullres), 0, 0, INTER_NEAREST);
	
	// initial gco problem for high resolution
	GCoptimizationGridGraph *gc_fullres = new GCoptimizationGridGraph(rep->colsSyn_fullres, rep->rowsSyn_fullres, rep->num_shiftXY);
	qDebug()<<"Only use pixel information for full resolution";
	gc_fullres->setDataCost( &dataFnborder_fullres);
	gc_fullres->setSmoothCost( &smoothFn1_fullres);
	// initialize using low resolution gco
	for (int i_l = 0; i_l < rep->numPixelSyn_fullres; i_l++)
	{
		gc_fullres->setLabel(i_l, rep->gcolabelSyn_fullres(gcNodes_fullres[i_l].y, gcNodes_fullres[i_l].x));
	}
	qDebug()<<"Before optimization energy is "<<gc_fullres->compute_energy();
	// the correct way ...
	gc_fullres->expansion(1);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
	qDebug()<<"after expansion energy is "<<gc_fullres->compute_energy(); // for efficiency reason only do one round of expansion
	//gc_fullres->swap(1);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
	//qDebug()<<"after swap energy is "<<gc_fullres->compute_energy();
	//gc_fullres->expansion(1);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
	//qDebug()<<"after expansion energy is "<<gc_fullres->compute_energy();
	//gc_fullres->swap(1);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
	//qDebug()<<"after swap energy is "<<gc_fullres->compute_energy();

	//------------------------------------------------------------------------------
	// Stitch the candidates by gc labels
	//------------------------------------------------------------------------------
	rep->imgSyn_fullres = Mat3b::zeros(rep->rowsSyn_fullres, rep->colsSyn_fullres);
	rep->gcolabelSyn_fullres = Mat1b::zeros(rep->rowsSyn_fullres, rep->colsSyn_fullres);
	rep->qimgSynlabelColor_fullres = new QImage(QSize(rep->colsSyn_fullres, rep->rowsSyn_fullres), QImage::Format_ARGB32);
	qDebug()<<"rep->colsSyn_fullres: "<<rep->colsSyn_fullres<<"rep->rowsSyn_fullres: "<<rep->rowsSyn_fullres;
	qDebug()<<"rep->colsInput_fullres: "<<rep->colsInput_fullres<<"rep->rowsInput_fullres: "<<rep->rowsInput_fullres;
	for ( int  i = 0; i < rep->numPixelSyn_fullres; i++ ){
		int label = gc_fullres->whatLabel(i);
		int newX = -gcLabels_fullres[label].x + gcNodes_fullres[i].x;
		int newY = -gcLabels_fullres[label].y + gcNodes_fullres[i].y;	
		if (newX >= 0 && newX < rep->colsInput_fullres && newY >= 0 && newY < rep->rowsInput_fullres){
			rep->imgSyn_fullres(gcNodes_fullres[i].y, gcNodes_fullres[i].x) = rep->imgInput_fullres(newY, newX);
			rep->gcolabelSyn_fullres(gcNodes_fullres[i].y, gcNodes_fullres[i].x) = label;
			int bblabel = rep->imgInputlabel_fullres(newY, newX) - 1;
			if (bblabel > -1)
			{
				QRgb value = qRgb(colorList[bblabel][0], colorList[bblabel][1], colorList[bblabel][2]);
				rep->qimgSynlabelColor_fullres->setPixel(gcNodes_fullres[i].x, gcNodes_fullres[i].y, value);
			}			
		}
	}
	*rep->qimgSyn_fullres = Mat2QImage(rep->imgSyn_fullres);
	rep->qlabelSyn_fullres = new QLabel;
	rep->qlabelSyn_fullres->setPixmap(QPixmap::fromImage(*rep->qimgSyn_fullres));
	imgDisp = new QGraphicsPixmapItem(QPixmap::fromImage(*rep->qimgSyn_fullres));

	rep->qlabelSynlabelColor_fullres = new QLabel;
	rep->qlabelSynlabelColor_fullres->setPixmap(QPixmap::fromImage(*rep->qimgSynlabelColor_fullres));

	scene->clear();
	scene->addItem(imgDisp);
	scene->setSceneRect(0, 0, rep->qimgSyn_fullres->width(), rep->qimgSyn_fullres->height());
	view = new QGraphicsView(scene);
	setCentralWidget(view);
	resize(rep->qimgSyn_fullres->width() + 10, rep->qimgSyn_fullres->height() + 50);	
}

void ImageViewer::synSampleMSL(){
	//------------------------------------------------------------------------------
	// gco at lower scale
	//------------------------------------------------------------------------------
	qDebug()<<"Synthesis using statistical sampling and Multi-resolution ";

	inputcols = rep->colsInput_scaled;
	inputrows = rep->rowsInput_scaled;
	syncols = rep->colsSyn_scaled;
	synrows = rep->rowsSyn_scaled;
	inputcols_fullres = rep->colsInput_fullres;
	inputrows_fullres = rep->rowsInput_fullres;

	rep->rowsSyn_fullres = rep->rowsSyn_scaled/scalerRes;
	rep->colsSyn_fullres = rep->colsSyn_scaled/scalerRes;
	syncols_fullres = rep->colsSyn_fullres;
	synrows_fullres = rep->rowsSyn_fullres;
	rep->numPixelSyn_fullres = syncols_fullres * synrows_fullres;

	rep->numPixelSyn_scaled = rep->rowsSyn_scaled * rep->colsSyn_scaled;
	rep->imgSynGray_scaled = Mat1b::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled); //or, rep->imgSynGray_scaled.create(rows, cols);
	rep->imgSynGray_fullres = Mat1b::zeros(rep->rowsSyn_fullres, rep->colsSyn_fullres);
	rep->gcolabelSyn_scaled = Mat1b::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled);
	
	// compute the candidate shift
	rep->num_shiftXY = rep->num_shiftX * rep->num_shiftY;
	rep->list_shiftXY_scaled.resize(rep->num_shiftXY);
	vector<Point2i>().swap(gcLabels); // free memory
	for (int i_s = 0; i_s < rep->num_shiftX; i_s++)
	{
		for (int j_s = 0; j_s < rep->num_shiftY; j_s++)
		{
			Point2i* temp = new Point2i( rep->list_shiftX_scaled[i_s], rep->list_shiftY_scaled[j_s]);
			rep->list_shiftXY_scaled[i_s * rep->num_shiftY + j_s] = *temp;
			gcLabels.push_back(rep->list_shiftXY_scaled[i_s * rep->num_shiftY + j_s]);
		}
	}

	rep->list_shiftXY_fullres.resize(rep->num_shiftXY);
	vector<Point2i>().swap(gcLabels_fullres); // free memory
	for (int i_s = 0; i_s < rep->num_shiftXY; i_s++)
	{
		rep->list_shiftXY_fullres[i_s].x = rep->list_shiftXY_scaled[i_s].x/scalerRes;
		rep->list_shiftXY_fullres[i_s].y = rep->list_shiftXY_scaled[i_s].y/scalerRes;
	}
	for (int i_s = 0; i_s < rep->num_shiftX; i_s++)
	{
		for (int j_s = 0; j_s < rep->num_shiftY; j_s++)
		{
			gcLabels_fullres.push_back(rep->list_shiftXY_fullres[i_s * rep->num_shiftY + j_s]);
		}
	}


	vector<Point2i>().swap(gcNodes); // free memory
	gcNodes = point2Node(rep->imgSynGray_scaled);
	vector<Point2i>().swap(gcNodes_fullres); // free memory
	gcNodes_fullres = point2Node(rep->imgSynGray_fullres);

	qDebug()<<"size of gcNodes_fullres: "<<gcNodes_fullres.size();

	qDebug()<<"*************************************";
	qDebug()<<"Candidate shifts for this round: ";
	qDebug()<<"*************************************";
	for (int i_s = 0; i_s < rep->num_shiftXY; i_s++){
		qDebug()<<rep->list_shiftXY_scaled[i_s].x<<rep->list_shiftXY_scaled[i_s].y;
	}

	//qDebug()<<"size of scaled synthesis image: row: "<<rep->rowsSyn_scaled<<", col: "<<rep->colsSyn_scaled;

	// set up the gco problem and solve it using swap
	GCoptimizationGridGraph *gc = new GCoptimizationGridGraph(rep->colsSyn_scaled, rep->rowsSyn_scaled, rep->num_shiftXY);

	if (synmode == 1)
	{
		qDebug()<<"Synthesis mode 1: image resizing";
		qDebug()<<"DataCost term: dataFnborder";
		gc->setDataCost( &dataFnborder);
		qDebug()<<"smoothness term: smoothFn1";
		gc->setSmoothCost( &smoothFn1);
	}
	else if (synmode == 2)
	{
		qDebug()<<"Synthesis mode 2: for interactive editing";
		qDebug()<<"DataCost term: dataFnborderGuide";
		gc->setDataCost( &dataFnborderGuide);
		qDebug()<<"smoothness term: smoothFn1";
		gc->setSmoothCost( &smoothFn1);
	}
	else if (synmode == 3)
	{
		qDebug()<<"Synthesis mode 3: for scribbling";
		qDebug()<<"DataCost term: dataFnborderScribble";
		gc->setDataCost( &dataFnborderScribble);
		qDebug()<<"smoothness term: smoothFn1";
		gc->setSmoothCost( &smoothFn1);
	}
	else if (synmode == 4)
	{
		qDebug()<<"Synthesis mode 4: switch to bb mode.";
		if (!flag_scribble && !flag_interactive)
		{
			qDebug()<<"DataCost term: dataFnborder";
			gc->setDataCost( &dataFnborder);
		}

		if (flag_scribble)
		{
			qDebug()<<"DataCost term: dataFnborderScribble";
			gc->setDataCost( &dataFnborderScribble);
		}

		if (flag_interactive)
		{
			qDebug()<<"DataCost term: dataFnborderGuide";
			gc->setDataCost( &dataFnborderGuide);
		}
		if (flag_constellation)
		{
			qDebug()<<"smoothness term: smoothFnConstellation3";
			gc->setSmoothCost(&smoothFnConstellation3);
		} 
		else
		{
			qDebug()<<"smoothness term: smoothFn3";
			gc->setSmoothCost( &smoothFn3);
		}
	}

	for (int i_l = 0; i_l < rep->numPixelSyn_scaled; i_l++)
	{
		gc->setLabel(i_l, ini_label);
	}

	qDebug()<<"Before optimization energy is "<<gc->compute_energy();

	//// the correct way ...
	gc->expansion(1);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
	qDebug()<<"after expansion energy is "<<gc->compute_energy();
	gc->swap(1);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
	qDebug()<<"after swap energy is "<<gc->compute_energy();
	gc->expansion(1);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
	qDebug()<<"after expansion energy is "<<gc->compute_energy();
	gc->swap(1);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
	qDebug()<<"after swap energy is "<<gc->compute_energy();


	//------------------------------------------------------------------------------
	// Stitch the candidates by gc labels
	//------------------------------------------------------------------------------
	rep->imgSyn_scaled = Mat3b::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled);
	qDebug()<<"rep->colsSyn_scaled: "<<rep->colsSyn_scaled<<"rep->rowsSyn_scaled: "<<rep->rowsSyn_scaled;
	qDebug()<<"rep->colsInput_scaled: "<<rep->colsInput_scaled<<"rep->rowsInput_scaled: "<<rep->rowsInput_scaled;
	for ( int  i = 0; i < rep->numPixelSyn_scaled; i++ ){
		int label = gc->whatLabel(i);
		int newX = -gcLabels[label].x + gcNodes[i].x;
		int newY = -gcLabels[label].y + gcNodes[i].y;	

		if (newX >= 0 && newX < rep->colsInput_scaled && newY >= 0 && newY < rep->rowsInput_scaled){
			rep->imgSyn_scaled(gcNodes[i].y, gcNodes[i].x) = rep->imgInput_scaled(newY, newX);
			rep->gcolabelSyn_scaled(gcNodes[i].y, gcNodes[i].x) = label;
		}
	}

	*rep->qimgSyn_scaled = Mat2QImage(rep->imgSyn_scaled);
	rep->qlabelSyn_scaled->setPixmap(QPixmap::fromImage(*rep->qimgSyn_scaled));
	delete gc;

	// compute the label image for synthesized image 
	rep->imgGuide_scaled = Mat1b::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled);
	for (int r = 0; r < rep->rowsSyn_scaled; r++)
	{
		for (int c = 0; c < rep->colsSyn_scaled; c++)
		{
			if ((c - rep->list_shiftXY_scaled[rep->gcolabelSyn_scaled(r, c)].x) < rep->colsInput_scaled && (c - rep->list_shiftXY_scaled[rep->gcolabelSyn_scaled(r, c)].x) >= 0)
			{
				if((r - rep->list_shiftXY_scaled[rep->gcolabelSyn_scaled(r, c)].y) < rep->rowsInput_scaled && (r - rep->list_shiftXY_scaled[rep->gcolabelSyn_scaled(r, c)].y) >= 0)
				{
					rep->imgGuide_scaled(r, c) = rep->imgInputlabel_scaled(r - rep->list_shiftXY_scaled[rep->gcolabelSyn_scaled(r, c)].y, c - rep->list_shiftXY_scaled[rep->gcolabelSyn_scaled(r, c)].x);
				}
			}
		}
	}

	//------------------------------------------------------------------------------
	// gco at high scale
	//------------------------------------------------------------------------------
	// well, it turns out that only a few shifts are not used. so do not need to remove anything from the old list
	rep->list_shiftXY_fullres.resize(rep->num_shiftXY);
	for (int i_s = 0; i_s < rep->num_shiftXY; i_s++)
	{
		rep->list_shiftXY_fullres[i_s].x = rep->list_shiftXY_scaled[i_s].x/scalerRes;
		rep->list_shiftXY_fullres[i_s].y = rep->list_shiftXY_scaled[i_s].y/scalerRes;
	}

	vector<Point2i> shift_local;
	shift_local.resize(4);
 //   shift_local[0].x = -4;
	//shift_local[0].y = -4;
	//shift_local[1].x = 4;
	//shift_local[1].y = -4;
	//shift_local[2].x = -4;
	//shift_local[2].y = 4;
	//shift_local[3].x = 4;
	//shift_local[3].y = 4;
	shift_local[0].x = -4;
	shift_local[0].y = -4;
	shift_local[1].x = 4;
	shift_local[1].y = -4;
	shift_local[2].x = -4;
	shift_local[2].y = 4;
	shift_local[3].x = 4;
	shift_local[3].y = 4;

	vector<Point2i>().swap(rep->list_shiftXY_MSL_fullres);
	rep->list_shiftXY_MSL_fullres.resize(rep->list_shiftXY_fullres.size() * 5);
	rep->num_shiftMSL = rep->num_shiftXY * 5;
	vector<Point2i>().swap(gcLabels_fullres); // free memory
	for (int i_s = 0; i_s < rep->list_shiftXY_fullres.size(); i_s++){
		rep->list_shiftXY_MSL_fullres[i_s * 5] = rep->list_shiftXY_fullres[i_s];
		gcLabels_fullres.push_back(rep->list_shiftXY_MSL_fullres[i_s * 5]);
		rep->list_shiftXY_MSL_fullres[i_s * 5 + 1] = rep->list_shiftXY_fullres[i_s] + shift_local[0];
		gcLabels_fullres.push_back(rep->list_shiftXY_MSL_fullres[i_s * 5 + 1]);
		rep->list_shiftXY_MSL_fullres[i_s * 5 + 2] = rep->list_shiftXY_fullres[i_s] + shift_local[1];
		gcLabels_fullres.push_back(rep->list_shiftXY_MSL_fullres[i_s * 5 + 2]);
		rep->list_shiftXY_MSL_fullres[i_s * 5 + 3] = rep->list_shiftXY_fullres[i_s] + shift_local[2];
		gcLabels_fullres.push_back(rep->list_shiftXY_MSL_fullres[i_s * 5 + 3]);
		rep->list_shiftXY_MSL_fullres[i_s * 5 + 4] = rep->list_shiftXY_fullres[i_s] + shift_local[3];
		gcLabels_fullres.push_back(rep->list_shiftXY_MSL_fullres[i_s * 5 + 4]);
	}


	for (int i_s = 0; i_s < rep->list_shiftXY_MSL_fullres.size(); i_s++)
	{
		qDebug()<<rep->list_shiftXY_MSL_fullres[i_s].x<<rep->list_shiftXY_MSL_fullres[i_s].y;
	}


	vector<Point2i>().swap(gcNodes_fullres); // free memory
	gcNodes_fullres = point2Node(rep->imgSynGray_fullres);

	// we need to record the low resolution label for each pixel
	gcGuide_MSL_fullres = Mat1b::zeros(rep->rowsSyn_fullres, rep->colsSyn_fullres);
	cv::resize(rep->gcolabelSyn_scaled, gcGuide_MSL_fullres, Size(rep->colsSyn_fullres,rep->rowsSyn_fullres), 0, 0, INTER_NEAREST);
	cv::resize(rep->gcolabelSyn_scaled, rep->gcolabelSyn_fullres, Size(rep->colsSyn_fullres,rep->rowsSyn_fullres), 0, 0, INTER_NEAREST);

	GCoptimizationGridGraph *gc_fullres = new GCoptimizationGridGraph(rep->colsSyn_fullres, rep->rowsSyn_fullres, rep->num_shiftMSL);
	qDebug()<<"Only use pixel information for full resolution";
	gc_fullres->setDataCost(&dataFnborder_MSL);
	if (synmode == 4)
	{
		gc_fullres->setSmoothCost(&smoothFn3_MSL);
		//gc_fullres->setSmoothCost(&smoothFn1_MSL);
	}
	else{
		gc_fullres->setSmoothCost(&smoothFn1_MSL);
	}
	
	//for (int i_l = 0; i_l < rep->numPixelSyn_fullres; i_l++)
	//{
	//	gc_fullres->setLabel(i_l, rep->gcolabelSyn_fullres(gcNodes_fullres[i_l].y, gcNodes_fullres[i_l].x) * 5);
	//}
	qDebug()<<"Before optimization energy is "<<gc_fullres->compute_energy();

	gc_fullres->expansion(1);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
	qDebug()<<"after expansion energy is "<<gc_fullres->compute_energy(); // for efficiency reason only do one round of expansion
	//gc_fullres->swap(1);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
	//qDebug()<<"after swap energy is "<<gc_fullres->compute_energy();
	//gc_fullres->expansion(1);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
	//qDebug()<<"after expansion energy is "<<gc_fullres->compute_energy();
	//gc_fullres->swap(1);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
	//qDebug()<<"after swap energy is "<<gc_fullres->compute_energy();

	//------------------------------------------------------------------------------
	// Stitch the candidates by gc labels
	//------------------------------------------------------------------------------
	rep->imgSyn_fullres = Mat3b::zeros(rep->rowsSyn_fullres, rep->colsSyn_fullres);
	rep->gcolabelSyn_fullres = Mat1b::zeros(rep->rowsSyn_fullres, rep->colsSyn_fullres);
	rep->qimgSynlabelColor_fullres = new QImage(QSize(rep->colsSyn_fullres, rep->rowsSyn_fullres), QImage::Format_ARGB32);
	qDebug()<<"rep->colsSyn_fullres: "<<rep->colsSyn_fullres<<"rep->rowsSyn_fullres: "<<rep->rowsSyn_fullres;
	qDebug()<<"rep->colsInput_fullres: "<<rep->colsInput_fullres<<"rep->rowsInput_fullres: "<<rep->rowsInput_fullres;
	for ( int  i = 0; i < rep->numPixelSyn_fullres; i++ ){
		int label = gc_fullres->whatLabel(i);
		int newX = -gcLabels_fullres[label].x + gcNodes_fullres[i].x;
		int newY = -gcLabels_fullres[label].y + gcNodes_fullres[i].y;	
		if (newX >= 0 && newX < rep->colsInput_fullres && newY >= 0 && newY < rep->rowsInput_fullres){
			rep->imgSyn_fullres(gcNodes_fullres[i].y, gcNodes_fullres[i].x) = rep->imgInput_fullres(newY, newX);
			rep->gcolabelSyn_fullres(gcNodes_fullres[i].y, gcNodes_fullres[i].x) = label;
			int bblabel = rep->imgInputlabel_fullres(newY, newX) - 1;
			if (bblabel > -1)
			{
				QRgb value = qRgb(colorList[bblabel][0], colorList[bblabel][1], colorList[bblabel][2]);
				rep->qimgSynlabelColor_fullres->setPixel(gcNodes_fullres[i].x, gcNodes_fullres[i].y, value);
			}			
		}
	}
	*rep->qimgSyn_fullres = Mat2QImage(rep->imgSyn_fullres);
	rep->qlabelSyn_fullres = new QLabel;
	rep->qlabelSyn_fullres->setPixmap(QPixmap::fromImage(*rep->qimgSyn_fullres));
	imgDisp = new QGraphicsPixmapItem(QPixmap::fromImage(*rep->qimgSyn_fullres));

	rep->qlabelSynlabelColor_fullres = new QLabel;
	rep->qlabelSynlabelColor_fullres->setPixmap(QPixmap::fromImage(*rep->qimgSynlabelColor_fullres));

	scene->clear();
	scene->addItem(imgDisp);
	scene->setSceneRect(0, 0, rep->qimgSyn_fullres->width(), rep->qimgSyn_fullres->height());
	view = new QGraphicsView(scene);
	setCentralWidget(view);
	resize(rep->qimgSyn_fullres->width() + 10, rep->qimgSyn_fullres->height() + 50);	


}

void ImageViewer::renderRepLabel(){
	renderRepLabelflag = 1 - renderRepLabelflag;
	if (renderRepLabelflag > 0)
	{	
		imgDisp = new QGraphicsPixmapItem(QPixmap::fromImage(*rep->qimgSynlabelColor_fullres));
		scene->clear();
		scene->addItem(imgDisp);
		scene->setSceneRect(0, 0, rep->qimgSynlabelColor_fullres->width(), rep->qimgSynlabelColor_fullres->height());
		view = new QGraphicsView(scene);
		setCentralWidget(view);
		resize(rep->qimgSynlabelColor_fullres->width() + 10, rep->qimgSynlabelColor_fullres->height() + 50);
	} 
	else
	{	
		imgDisp = new QGraphicsPixmapItem(QPixmap::fromImage(*rep->qimgSyn_fullres));
		scene->clear();
		scene->addItem(imgDisp);
		scene->setSceneRect(0, 0, rep->qimgSyn_fullres->width(), rep->qimgSyn_fullres->height());
		view = new QGraphicsView(scene);
		setCentralWidget(view);
		resize(rep->qimgSyn_fullres->width() + 10, rep->qimgSyn_fullres->height() + 50);
	}
}

void ImageViewer::renderRepBB(){
	if (rep->num_shiftX == 1)
		return;
	
	flag_syn = true;
	estimationSW();
	qDebug()<<"render repetitive BB in the synthesized image.";
	imgDisp = new QGraphicsPixmapItem(QPixmap::fromImage(*rep->qimgSyn_fullres));
	scene->clear();
	scene->addItem(imgDisp);
	scene->setSceneRect(0, 0, rep->qimgSyn_fullres->width(), rep->qimgSyn_fullres->height());
	view = new QGraphicsView(scene);
	setCentralWidget(view);
	setWindowTitle(tr("ImageSyn"));
	resize(rep->qimgSyn_fullres->width() + 10, rep->qimgSyn_fullres->height() + 50);

	//get repetitive building blocks
	vector<vector<BBItem*>>().swap(bbitem);
	bbitem.resize(rep->numRep);
	for (int i_s = 0; i_s < rep->rec_bb_list.size(); i_s++)
	{
		for (int i_item = 0; i_item < rep->rec_bb_list[i_s].size(); i_item++)
		{
			int i_rep = rep->rec_bb_list[i_s][i_item].first;
			int j_rep = rep->rec_bb_list[i_s][i_item].second;
			int X = rep->repX_fullres[i_rep][j_rep] + rep->list_shiftXY_fullres[rep->rec_shift_list[i_s]].x;
			int Y = rep->repY_fullres[i_rep][j_rep] + rep->list_shiftXY_fullres[rep->rec_shift_list[i_s]].y;
			int W = rep->repW_fullres[i_rep][j_rep];
			int H = rep->repH_fullres[i_rep][j_rep];

			QPolygon polygon;
			polygon << QPoint(X, Y)
				<< QPoint(X, Y + H)
				<< QPoint(X + W, Y + H)
				<< QPoint(X + W, Y);
			BBItem *item1 = new BBItem(i_s, i_item);
			item1->setPolygon(polygon);
			item1->idx_s = rep->rec_shift_list[i_s];
			item1->x_start = 0;
			item1->y_start = 0;
			item1->x_shift = rep->list_shiftXY_scaled[rep->rec_shift_list[i_s]].x;
			item1->y_shift = rep->list_shiftXY_scaled[rep->rec_shift_list[i_s]].y;
			item1->x_shift_ori = rep->list_shiftXY_scaled[rep->rec_shift_list[i_s]].x;
			item1->y_shift_ori = rep->list_shiftXY_scaled[rep->rec_shift_list[i_s]].y;
			item1->bb_type = i_rep;
			item1->bb_idx = j_rep;
			item1->x_fullres = rep->repX_fullres[i_rep][j_rep];
			item1->y_fullres = rep->repY_fullres[i_rep][j_rep];
			item1->w_fullres = rep->repW_fullres[i_rep][j_rep];
			item1->h_fullres = rep->repH_fullres[i_rep][j_rep];
			item1->x_scaled = rep->repX_scaled[i_rep][j_rep];
			item1->y_scaled = rep->repY_scaled[i_rep][j_rep];
			item1->w_scaled = rep->repW_scaled[i_rep][j_rep];
			item1->h_scaled = rep->repH_scaled[i_rep][j_rep];
			item1->idx2rec_bb_list = Point2i(i_s, i_item);
			bbitem[i_rep].push_back(item1);
			int cur_idx = bbitem[i_rep].size() - 1;
			bbitem[i_rep][cur_idx]->setPen(pen);
			bbitem[i_rep][cur_idx]->setBrush( QColor(colorList[i_rep][0], colorList[i_rep][1], colorList[i_rep][2]) );
			bbitem[i_rep][cur_idx]->setZValue(qrand()%256);
			bbitem[i_rep][cur_idx]->setOpacity(0.75);
		}
	}

	for (int i_rep = 0; i_rep < bbitem.size(); i_rep++)
	{
		for (int j_rep = 0; j_rep < bbitem[i_rep].size(); j_rep++)
		{
			scene->addItem(bbitem[i_rep][j_rep]);
		}
	}
}

void ImageViewer::estimationSW(){
	qDebug()<<"Estimation: sliding window bb reconstruction ...";
	vector<int>().swap(rep->rec_shift_list);
	vector<vector<pair<int, int>>>().swap(rep->rec_bb_list);
	Mat1b imgGuide_in = Mat1b::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled);
	Mat1b imgGuide_out = Mat1b::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled);
	rep->imgGuide_scaled.copyTo(imgGuide_in(Rect(0, 0, rep->imgGuide_scaled.cols, rep->imgGuide_scaled.rows)));

	for (int i_iter = 0; i_iter < max_iter_rec; i_iter++)
	{
		vector<double> score;
		vector<double> area;
		score.resize(rep->num_shiftXY);
		fill(score.begin(), score.end(), 0);
		area.resize(rep->num_shiftXY);
		fill(area.begin(), area.end(), 0);
		vector<vector<pair<int, int>>> bb_list;
		bb_list.resize(rep->num_shiftXY);

		for (int i_s = 0; i_s < rep->num_shiftXY; i_s++)
		{
			bb_list[i_s].resize(0);
			for (int i_rep = 0; i_rep < rep->numRep; i_rep++)
			{
				for (int j_rep = 0; j_rep < rep->sizeRep[i_rep]; j_rep++)
				{

					Range r_y = Range(rep->repY_scaled[i_rep][j_rep] + rep->list_shiftXY_scaled[i_s].y, rep->repY_scaled[i_rep][j_rep] + rep->repH_scaled[i_rep][j_rep] + rep->list_shiftXY_scaled[i_s].y);
					Range r_x = Range(rep->repX_scaled[i_rep][j_rep] + rep->list_shiftXY_scaled[i_s].x, rep->repX_scaled[i_rep][j_rep] + rep->repW_scaled[i_rep][j_rep] + rep->list_shiftXY_scaled[i_s].x);	
					r_y.start = min(rep->rowsSyn_scaled - 1, max(0, r_y.start));
					r_y.end = min(rep->rowsSyn_scaled - 1, max(0, r_y.end));
					r_x.start = min(rep->colsSyn_scaled - 1, max(0, r_x.start));
					r_x.end = min(rep->colsSyn_scaled - 1, max(0, r_x.end));

					
					if (r_x.end - r_x.start	> 0 & r_y.end - r_y.start > 0)
					{
						double cover_in;
						double cover_out;
						int count_in;
						int count_out;

						Mat1b tempM_in;
						imgGuide_in(r_y, r_x).copyTo(tempM_in);
						tempM_in = tempM_in == (i_rep + 1);
						count_in = countNonZero(tempM_in);
						cover_in = (double)count_in/(double)((rep->repH_scaled[i_rep][j_rep] + 1) * (rep->repW_scaled[i_rep][j_rep] + 1));

						Mat1b tempM_out;
						imgGuide_out(r_y, r_x).copyTo(tempM_out);
						tempM_out = tempM_out == (i_rep + 1);
						count_out = countNonZero(tempM_out);
						cover_out = (double)count_out/(double)((rep->repH_scaled[i_rep][j_rep] + 1) * (rep->repW_scaled[i_rep][j_rep] + 1));

						if( cover_in > global_cover_in_thresh && cover_out < global_cover_out_thresh){
							score[i_s] += 1;
							bb_list[i_s].push_back(make_pair(i_rep,j_rep));
							area[i_s] += cover_in;
						} 
						else
						{

						}
					}
					else{
						//qDebug()<<r_x.start<<r_x.end<<r_y.start<<r_y.end;
					}

				}
			}
		}

		auto maxc = max_element(area.begin(), area.end());
		int maxi = distance(area.begin(), maxc);
		double dmaxc = score[maxi];
		if (dmaxc > 0)
		{
			rep->rec_shift_list.push_back(maxi);
			rep->rec_bb_list.push_back(bb_list[maxi]);
			for (int i_rep = 0; i_rep < rep->rec_bb_list[i_iter].size(); i_rep++)
			{
				Range r_y = Range(rep->repY_scaled[rep->rec_bb_list[i_iter][i_rep].first][rep->rec_bb_list[i_iter][i_rep].second] + rep->list_shiftXY_scaled[rep->rec_shift_list[i_iter]].y, rep->repY_scaled[rep->rec_bb_list[i_iter][i_rep].first][rep->rec_bb_list[i_iter][i_rep].second] + rep->repH_scaled[rep->rec_bb_list[i_iter][i_rep].first][rep->rec_bb_list[i_iter][i_rep].second] + rep->list_shiftXY_scaled[rep->rec_shift_list[i_iter]].y);	
				Range r_x = Range(rep->repX_scaled[rep->rec_bb_list[i_iter][i_rep].first][rep->rec_bb_list[i_iter][i_rep].second] + rep->list_shiftXY_scaled[rep->rec_shift_list[i_iter]].x, rep->repX_scaled[rep->rec_bb_list[i_iter][i_rep].first][rep->rec_bb_list[i_iter][i_rep].second] + rep->repW_scaled[rep->rec_bb_list[i_iter][i_rep].first][rep->rec_bb_list[i_iter][i_rep].second] + rep->list_shiftXY_scaled[rep->rec_shift_list[i_iter]].x);	
				r_y.start = min(rep->rowsSyn_scaled - 1, max(0, r_y.start));
				r_x.start = min(rep->colsSyn_scaled - 1, max(0, r_x.start));
				r_y.end = min(rep->rowsSyn_scaled - 1, max(0, r_y.end));
				r_x.end = min(rep->colsSyn_scaled - 1, max(0, r_x.end));
				imgGuide_in(r_y, r_x) = 0;
				imgGuide_out(r_y, r_x) = rep->rec_bb_list[i_iter][i_rep].first + 1;
			}
		}
		else{
			break;
		}
	}

	//------------------------------------------------------------
	// prepare bb for maximization 
	//------------------------------------------------------------
	//get repetitive building blocks
	vector<vector<BBItem*>>().swap(bbitem);
	bbitem.resize(rep->numRep);
	for (int i_s = 0; i_s < rep->rec_bb_list.size(); i_s++)
	{
		for (int i_item = 0; i_item < rep->rec_bb_list[i_s].size(); i_item++)
		{
			int i_rep = rep->rec_bb_list[i_s][i_item].first;
			int j_rep = rep->rec_bb_list[i_s][i_item].second;
			int X = rep->repX_fullres[i_rep][j_rep] + rep->list_shiftXY_fullres[rep->rec_shift_list[i_s]].x;
			int Y = rep->repY_fullres[i_rep][j_rep] + rep->list_shiftXY_fullres[rep->rec_shift_list[i_s]].y;
			int W = rep->repW_fullres[i_rep][j_rep];
			int H = rep->repH_fullres[i_rep][j_rep];

			QPolygon polygon;
			polygon << QPoint(X, Y)
				<< QPoint(X, Y + H)
				<< QPoint(X + W, Y + H)
				<< QPoint(X + W, Y);
			BBItem *item1 = new BBItem(i_s, i_item);
			item1->setPolygon(polygon);
			item1->idx_s = rep->rec_shift_list[i_s];
			item1->x_start = 0;
			item1->y_start = 0;
			item1->x_shift = rep->list_shiftXY_scaled[rep->rec_shift_list[i_s]].x;
			item1->y_shift = rep->list_shiftXY_scaled[rep->rec_shift_list[i_s]].y;
			item1->x_shift_ori = rep->list_shiftXY_scaled[rep->rec_shift_list[i_s]].x;
			item1->y_shift_ori = rep->list_shiftXY_scaled[rep->rec_shift_list[i_s]].y;
			item1->bb_type = i_rep;
			item1->bb_idx = j_rep;
			item1->x_fullres = rep->repX_fullres[i_rep][j_rep];
			item1->y_fullres = rep->repY_fullres[i_rep][j_rep];
			item1->w_fullres = rep->repW_fullres[i_rep][j_rep];
			item1->h_fullres = rep->repH_fullres[i_rep][j_rep];
			item1->x_scaled = rep->repX_scaled[i_rep][j_rep];
			item1->y_scaled = rep->repY_scaled[i_rep][j_rep];
			item1->w_scaled = rep->repW_scaled[i_rep][j_rep];
			item1->h_scaled = rep->repH_scaled[i_rep][j_rep];
			bbitem[i_rep].push_back(item1);
			int cur_idx = bbitem[i_rep].size() - 1;
			bbitem[i_rep][cur_idx]->setPen(pen);
			bbitem[i_rep][cur_idx]->setBrush( QColor(colorList[i_rep][0], colorList[i_rep][1], colorList[i_rep][2]) );
			bbitem[i_rep][cur_idx]->setZValue(qrand()%256);
			bbitem[i_rep][cur_idx]->setOpacity(0.75);
		}
	}
	// update list of gcLabels (because it is coupled with rep->list_shiftXY_scaled)
	vector<Point2i>().swap(gcLabels);
	for (int i_s = 0; i_s < rep->list_shiftXY_scaled.size(); i_s++)
	{
		gcLabels.push_back(Point2i( rep->list_shiftXY_scaled[i_s].x, rep->list_shiftXY_scaled[i_s].y));
	}
	//update gcGuide map for computing data cost
	gcGuide = Mat1d::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled);
	gcGuide = gcGuide - 1;
	gcGuideMask = Mat1d::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled);

	int numRecBB = 0;
	for (int i_rep = 0; i_rep < bbitem.size(); i_rep++)
	{
		for (int j_rep = 0; j_rep < bbitem[i_rep].size(); j_rep++)
		{
			//update gcLabel, gcLabelinterX, gcLabelinterY and gcGuide
			Range r_y = Range(rep->repY_scaled[bbitem[i_rep][j_rep]->bb_type][bbitem[i_rep][j_rep]->bb_idx] + bbitem[i_rep][j_rep]->y_shift, rep->repY_scaled[bbitem[i_rep][j_rep]->bb_type][bbitem[i_rep][j_rep]->bb_idx] + rep->repH_scaled[bbitem[i_rep][j_rep]->bb_type][bbitem[i_rep][j_rep]->bb_idx] + bbitem[i_rep][j_rep]->y_shift);
			Range r_x = Range(rep->repX_scaled[bbitem[i_rep][j_rep]->bb_type][bbitem[i_rep][j_rep]->bb_idx] + bbitem[i_rep][j_rep]->x_shift, rep->repX_scaled[bbitem[i_rep][j_rep]->bb_type][bbitem[i_rep][j_rep]->bb_idx] + rep->repW_scaled[bbitem[i_rep][j_rep]->bb_type][bbitem[i_rep][j_rep]->bb_idx] + bbitem[i_rep][j_rep]->x_shift);	
			r_y.start = min(rep->rowsSyn_scaled - 1, max(0, r_y.start));
			r_x.start = min(rep->colsSyn_scaled - 1, max(0, r_x.start));
			r_y.end = min(rep->rowsSyn_scaled - 1, max(0, r_y.end));
			r_x.end = min(rep->colsSyn_scaled - 1, max(0, r_x.end));
			// update gcGuide
			auto loc = std::find(rep->list_shiftXY_scaled.begin(), rep->list_shiftXY_scaled.end(), Point2i(bbitem[i_rep][j_rep]->x_shift, bbitem[i_rep][j_rep]->y_shift));
			int idx_shift = loc - rep->list_shiftXY_scaled.begin();
			gcGuide(r_y, r_x) = idx_shift;
			gcGuideMask(r_y, r_x) = 1;
			numRecBB++;
		}
	}
}

void ImageViewer::estimationConfig(){
	qDebug()<<"Estimation: reconfigure building blocks ...";
	vector<int>().swap(rep->config_shift_list);
	vector<vector<pair<int, int>>>().swap(rep->config_bb_list);

	// initialize the configured shift list as the first entry in rec_shift_list
	rep->config_shift_list.resize(1);
	rep->config_shift_list[0] = rep->rec_shift_list[0];
	rep->config_bb_list.resize(1);
	rep->config_bb_list[0].resize(rep->rec_bb_list[0].size());
	for (int i = 0; i < rep->rec_bb_list[0].size(); i++)
	{
		rep->config_bb_list[0][i] = rep->rec_bb_list[0][i];
	}

	vector<int> remaining_shift_list;
	vector<vector<pair<int, int>>> remanining_bb_list;
	remaining_shift_list.resize(rep->rec_shift_list.size() - 1);
	remanining_bb_list.resize(rep->rec_shift_list.size() - 1);
	for (int i = 0; i < rep->rec_shift_list.size() - 1; i++)
	{
		remaining_shift_list[i] = rep->rec_shift_list[i + 1];
		remanining_bb_list[i].resize(rep->rec_bb_list[i + 1].size());
		for (int j = 0; j < rep->rec_bb_list[i + 1].size(); j++)
		{
			remanining_bb_list[i][j] = rep->rec_bb_list[i + 1][j];
		}	
	}

	int count_step = 1;
	while (count_step < rep->rec_shift_list.size()){
		vector<int>().swap(rep->sizeVote);
		vector<vector<int>>().swap(rep->voteX_scaled);
		vector<vector<int>>().swap(rep->voteY_scaled);
		rep->voteX_scaled.resize(rep->numRep);
		rep->voteY_scaled.resize(rep->numRep);
		vector<vector<int>> tempvoteX_scaled;
		vector<vector<int>> tempvoteY_scaled;
		tempvoteX_scaled.resize(rep->numRep);
		tempvoteY_scaled.resize(rep->numRep);

		vector<vector<BBItem*>> InItem;
		vector<vector<BBItem*>>().swap(InItem);
		InItem.resize(rep->numRep);

		//------------------------------------------------------------
		// collect all bb items in the current rep->config_shift_list
		// and vote
		//------------------------------------------------------------
		for (int i_s = 0; i_s < rep->config_shift_list.size(); i_s++)
		{
			for (int i_item = 0; i_item < rep->config_bb_list[i_s].size(); i_item++)
			{
				int i_rep = rep->config_bb_list[i_s][i_item].first;
				int j_rep = rep->config_bb_list[i_s][i_item].second;
				int X = rep->repX_scaled[i_rep][j_rep] + rep->list_shiftXY_scaled[rep->config_shift_list[i_s]].x;
				int Y = rep->repY_scaled[i_rep][j_rep] + rep->list_shiftXY_scaled[rep->config_shift_list[i_s]].y;
				for (int i_v = 0; i_v < rep->sizeCooC[i_rep]; i_v++)
				{
					int vX = X + rep->coocX_scaled[i_rep][i_v];
					int vY = Y + rep->coocY_scaled[i_rep][i_v];
					tempvoteX_scaled[i_rep].push_back(vX);
					tempvoteY_scaled[i_rep].push_back(vY);
				}
			}
		}

		for (int i_v = 0; i_v < tempvoteX_scaled.size(); i_v++)
		{
			for (int j_v = 0; j_v < tempvoteX_scaled[i_v].size(); j_v++)
			{
				if (tempvoteX_scaled[i_v][j_v] >= 0 && tempvoteX_scaled[i_v][j_v] < rep->colsSyn_scaled && tempvoteY_scaled[i_v][j_v] >= 0 && tempvoteY_scaled[i_v][j_v] < rep->rowsSyn_scaled)
				{
					//
					rep->voteX_scaled[i_v].push_back(tempvoteX_scaled[i_v][j_v]);
					rep->voteY_scaled[i_v].push_back(tempvoteY_scaled[i_v][j_v]);
				} 
				else
				{
				}
			}
		}


		//------------------------------------------------------------
		// find the next closet shift in the remaining rep->rec_shift_list
		//------------------------------------------------------------
		// find each element in remaining_shift_list a closest match in config_shift_list to be found 
		vector<int> min_distr2c;
		min_distr2c.resize(remaining_shift_list.size());
		for (int i_r = 0; i_r < remaining_shift_list.size(); i_r++)
		{
			// compute distance from remaining_shift_list[i_r] to config_shift_list
			vector<int> dist;
			dist.resize(rep->config_shift_list.size());
			for (int i_c = 0; i_c < rep->config_shift_list.size(); i_c++)
			{
				dist[i_c] = abs(rep->list_shiftXY_scaled[rep->config_shift_list[i_c]].x - rep->list_shiftXY_scaled[remaining_shift_list[i_r]].x) + abs(rep->list_shiftXY_scaled[rep->config_shift_list[i_c]].y - rep->list_shiftXY_scaled[remaining_shift_list[i_r]].y);
			}
			// find the smallest entry in dist;
			auto minc = min_element(dist.begin(), dist.end());
			int mini = distance(dist.begin(), minc);
			min_distr2c[i_r] = dist[mini];
		}
		auto minc = min_element(min_distr2c.begin(), min_distr2c.end());
		int id_NNshift = distance(min_distr2c.begin(), minc);

		// add this shift to config_shift_list
		rep->config_shift_list.push_back(remaining_shift_list[id_NNshift]);
		rep->config_bb_list.push_back(remanining_bb_list[id_NNshift]);

		// remove this shift from the remaining list
		remaining_shift_list.erase(remaining_shift_list.begin() + id_NNshift);
		remanining_bb_list.erase(remanining_bb_list.begin() + id_NNshift);

		//------------------------------------------------------------
		// update the shift
		//------------------------------------------------------------
		// the idea is to shift config_shift_list[count_step] around so it it has the strongest response to rep->voteX_scaled and rep->voteY_scaled
		vector<int> response;
		vector<Point2i> shift_record;
		response.resize((2 * step_reconfig + 1) * (2 * step_reconfig + 1));
		shift_record.resize((2 * step_reconfig + 1) * (2 * step_reconfig + 1));
		for (int i_step = -step_reconfig; i_step < step_reconfig; i_step++)
		{
			for (int j_step = -step_reconfig; j_step < step_reconfig; j_step++)
			{
				int idx = (2 * step_reconfig + 1) * (step_reconfig + i_step) + step_reconfig + j_step;
				// compute the current response with i_step 
				for (int i_item = 0; i_item < rep->config_bb_list[count_step].size(); i_item++)
				{
					shift_record[idx].x = i_step;
					shift_record[idx].y = j_step;
					int i_rep = rep->config_bb_list[count_step][i_item].first;
					int j_rep = rep->config_bb_list[count_step][i_item].second;
					int X = rep->repX_scaled[i_rep][j_rep] + rep->list_shiftXY_scaled[rep->config_shift_list[count_step]].x + i_step;
					int Y = rep->repY_scaled[i_rep][j_rep] + rep->list_shiftXY_scaled[rep->config_shift_list[count_step]].y + j_step;
					// we want to see how many response does this bb get from rep->voteX_scaled and rep->voteY_scaled
					for (int i_v = 0; i_v < rep->voteX_scaled[i_rep].size(); i_v++)
					{
						if (abs(rep->voteX_scaled[i_rep][i_v] - X) < 1 && abs(rep->voteY_scaled[i_rep][i_v] - Y) < 1)
						{
							response[idx] += 1;
							break;
						}
					}
				}
			}
		}

		auto maxs = max_element(response.begin(), response.end());
		int id_Configshift = distance(response.begin(), maxs);

		// need to see if this is a trivial movement
		if (response[id_Configshift]>response[(2 * step_reconfig + 1) * step_reconfig + step_reconfig])
		{
			rep->list_shiftXY_scaled[rep->config_shift_list[count_step]].x += shift_record[id_Configshift].x;
			rep->list_shiftXY_scaled[rep->config_shift_list[count_step]].y += shift_record[id_Configshift].y;

			rep->list_shiftXY_fullres[rep->config_shift_list[count_step]].x += shift_record[id_Configshift].x/scalerRes;
			rep->list_shiftXY_fullres[rep->config_shift_list[count_step]].y += shift_record[id_Configshift].y/scalerRes;
		} 
		else
		{
			// do nothing for the trivial movement
		}
		count_step += 1;
	}

	//------------------------------------------------------------
	//A trick here, to sort config_shift_list and config_bb_list by size, so in the later clean up stage we will keep large pattern as a whole
	//------------------------------------------------------------
	vector<pair<int, int>> temp4sort;
	temp4sort.resize(rep->config_shift_list.size());
	for (int i_s = 0; i_s < rep->config_shift_list.size(); i_s++)
	{
		temp4sort[i_s].first = rep->config_bb_list[i_s].size();
		temp4sort[i_s].second = i_s;
	}
	for (int i_s = 0; i_s < temp4sort.size(); i_s++)
	{
		//qDebug()<<temp4sort[i_s].first<<temp4sort[i_s].second;
	}
	std::sort(temp4sort.begin(),temp4sort.end(), sort_pred());
	for (int i_s = 0; i_s < temp4sort.size(); i_s++)
	{
		//qDebug()<<temp4sort[i_s].first<<temp4sort[i_s].second;
	}
	vector<int> tempconfig_shift_list;
	vector<vector<pair<int, int>>> tempconfig_bb_list;
	tempconfig_shift_list.resize(rep->config_shift_list.size());
	tempconfig_bb_list.resize(rep->config_shift_list.size());
	for (int i_s = 0; i_s < temp4sort.size(); i_s++)
	{
		tempconfig_shift_list[i_s] = rep->config_shift_list[temp4sort[i_s].second];
		tempconfig_bb_list[i_s] = rep->config_bb_list[temp4sort[i_s].second];
	}
	vector<int>().swap(rep->config_shift_list);
	vector<vector<pair<int, int>>>().swap(rep->config_bb_list);
	rep->config_shift_list = tempconfig_shift_list;
	rep->config_bb_list = tempconfig_bb_list;

	//------------------------------------------------------------
	//Remove overlapping bb
	//------------------------------------------------------------
	gcGuideMask = Mat1d::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled);
	vector<vector<int>> list_remove;
	list_remove.resize(rep->config_shift_list.size());

	for (int i_s = 0; i_s < rep->config_shift_list.size(); i_s++)
	{
		for (int i_item = 0; i_item < rep->config_bb_list[i_s].size(); i_item++)
		{
			int i_rep = rep->config_bb_list[i_s][i_item].first;
			int j_rep = rep->config_bb_list[i_s][i_item].second;

			// need to check if this item overlaps with other items
			Range r_y = Range(rep->repY_scaled[i_rep][j_rep] + rep->list_shiftXY_scaled[rep->config_shift_list[i_s]].y, rep->repY_scaled[i_rep][j_rep] + rep->list_shiftXY_scaled[rep->config_shift_list[i_s]].y + rep->repH_scaled[i_rep][j_rep]);	
			Range r_x = Range(rep->repX_scaled[i_rep][j_rep] + rep->list_shiftXY_scaled[rep->config_shift_list[i_s]].x, rep->repX_scaled[i_rep][j_rep] + rep->list_shiftXY_scaled[rep->config_shift_list[i_s]].x + rep->repW_scaled[i_rep][j_rep]);	

			r_y.start = min(rep->rowsSyn_scaled - 1, max(0, r_y.start));
			r_x.start = min(rep->colsSyn_scaled - 1, max(0, r_x.start));
			r_y.end = min(rep->rowsSyn_scaled - 1, max(0, r_y.end));
			r_x.end = min(rep->colsSyn_scaled - 1, max(0, r_x.end));

			int count_overlap = countNonZero(gcGuideMask(r_y, r_x));
			double ratio_overlap = (double)count_overlap/(double)((rep->repH_scaled[i_rep][j_rep] + 1) * (rep->repW_scaled[i_rep][j_rep] + 1));
			//------------------------------------------------------------
			// threshold needs to be improved
			//------------------------------------------------------------
			if (ratio_overlap < 0.01){
				gcGuideMask(r_y, r_x) = 1;
			}
			else
			{
				list_remove[i_s].push_back(i_item);
			}
		}
	}

	// update bb
	for (int i_rep = 0; i_rep < list_remove.size(); i_rep++)
	{
		for (int j_rep = list_remove[i_rep].size() - 1; j_rep > -1; j_rep--)
		{
			rep->config_bb_list[i_rep][list_remove[i_rep][j_rep]] = rep->config_bb_list[i_rep].back();
			rep->config_bb_list[i_rep].pop_back();
			list_remove[i_rep].pop_back();
		}
	}

	// clear rep->rec_shift_list and rep->rec_bb_list
	vector<int>().swap(rep->rec_shift_list);
	vector<vector<pair<int, int>>>().swap(rep->rec_bb_list);
	rep->rec_shift_list = rep->config_shift_list;
	rep->rec_bb_list = rep->config_bb_list;


	//------------------------------------------------------------
	// prepare bb for maximization 
	//------------------------------------------------------------
	//get repetitive building blocks
	vector<vector<BBItem*>>().swap(bbitem);
	bbitem.resize(rep->numRep);
	for (int i_s = 0; i_s < rep->rec_bb_list.size(); i_s++)
	{
		for (int i_item = 0; i_item < rep->rec_bb_list[i_s].size(); i_item++)
		{
			int i_rep = rep->rec_bb_list[i_s][i_item].first;
			int j_rep = rep->rec_bb_list[i_s][i_item].second;
			int X = rep->repX_fullres[i_rep][j_rep] + rep->list_shiftXY_fullres[rep->rec_shift_list[i_s]].x;
			int Y = rep->repY_fullres[i_rep][j_rep] + rep->list_shiftXY_fullres[rep->rec_shift_list[i_s]].y;
			int W = rep->repW_fullres[i_rep][j_rep];
			int H = rep->repH_fullres[i_rep][j_rep];

			QPolygon polygon;
			polygon << QPoint(X, Y)
				<< QPoint(X, Y + H)
				<< QPoint(X + W, Y + H)
				<< QPoint(X + W, Y);
			BBItem *item1 = new BBItem(i_s, i_item);
			item1->setPolygon(polygon);
			item1->idx_s = rep->rec_shift_list[i_s];
			item1->x_start = 0;
			item1->y_start = 0;
			item1->x_shift = rep->list_shiftXY_scaled[rep->rec_shift_list[i_s]].x;
			item1->y_shift = rep->list_shiftXY_scaled[rep->rec_shift_list[i_s]].y;
			item1->x_shift_ori = rep->list_shiftXY_scaled[rep->rec_shift_list[i_s]].x;
			item1->y_shift_ori = rep->list_shiftXY_scaled[rep->rec_shift_list[i_s]].y;
			item1->bb_type = i_rep;
			item1->bb_idx = j_rep;
			item1->x_fullres = rep->repX_fullres[i_rep][j_rep];
			item1->y_fullres = rep->repY_fullres[i_rep][j_rep];
			item1->w_fullres = rep->repW_fullres[i_rep][j_rep];
			item1->h_fullres = rep->repH_fullres[i_rep][j_rep];
			item1->x_scaled = rep->repX_scaled[i_rep][j_rep];
			item1->y_scaled = rep->repY_scaled[i_rep][j_rep];
			item1->w_scaled = rep->repW_scaled[i_rep][j_rep];
			item1->h_scaled = rep->repH_scaled[i_rep][j_rep];
			item1->idx2rec_bb_list = Point2i(i_s, i_item);
			bbitem[i_rep].push_back(item1);
			int cur_idx = bbitem[i_rep].size() - 1;
			bbitem[i_rep][cur_idx]->setPen(pen);
			bbitem[i_rep][cur_idx]->setBrush( QColor(colorList[i_rep][0], colorList[i_rep][1], colorList[i_rep][2]) );
			bbitem[i_rep][cur_idx]->setZValue(qrand()%256);
			bbitem[i_rep][cur_idx]->setOpacity(0.75);

		}
	}
	// update list of gcLabels (because it is coupled with rep->list_shiftXY_scaled)
	vector<Point2i>().swap(gcLabels);
	for (int i_s = 0; i_s < rep->list_shiftXY_scaled.size(); i_s++)
	{
		gcLabels.push_back(Point2i( rep->list_shiftXY_scaled[i_s].x, rep->list_shiftXY_scaled[i_s].y));
	}
	//update gcGuide map for computing data cost
	gcGuide = Mat1d::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled);
	gcGuide = gcGuide - 1;
	gcGuideMask = Mat1d::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled);

	int numRecBB = 0;
	for (int i_rep = 0; i_rep < bbitem.size(); i_rep++)
	{
		for (int j_rep = 0; j_rep < bbitem[i_rep].size(); j_rep++)
		{
			//update gcLabel, gcLabelinterX, gcLabelinterY and gcGuide
			Range r_y = Range(rep->repY_scaled[bbitem[i_rep][j_rep]->bb_type][bbitem[i_rep][j_rep]->bb_idx] + bbitem[i_rep][j_rep]->y_shift, rep->repY_scaled[bbitem[i_rep][j_rep]->bb_type][bbitem[i_rep][j_rep]->bb_idx] + rep->repH_scaled[bbitem[i_rep][j_rep]->bb_type][bbitem[i_rep][j_rep]->bb_idx] + bbitem[i_rep][j_rep]->y_shift);
			Range r_x = Range(rep->repX_scaled[bbitem[i_rep][j_rep]->bb_type][bbitem[i_rep][j_rep]->bb_idx] + bbitem[i_rep][j_rep]->x_shift, rep->repX_scaled[bbitem[i_rep][j_rep]->bb_type][bbitem[i_rep][j_rep]->bb_idx] + rep->repW_scaled[bbitem[i_rep][j_rep]->bb_type][bbitem[i_rep][j_rep]->bb_idx] + bbitem[i_rep][j_rep]->x_shift);	
			r_y.start = min(rep->rowsSyn_scaled - 1, max(0, r_y.start));
			r_x.start = min(rep->colsSyn_scaled - 1, max(0, r_x.start));
			r_y.end = min(rep->rowsSyn_scaled - 1, max(0, r_y.end));
			r_x.end = min(rep->colsSyn_scaled - 1, max(0, r_x.end));
			// update gcGuide
			auto loc = std::find(rep->list_shiftXY_scaled.begin(), rep->list_shiftXY_scaled.end(), Point2i(bbitem[i_rep][j_rep]->x_shift, bbitem[i_rep][j_rep]->y_shift));
			int idx_shift = loc - rep->list_shiftXY_scaled.begin();
			gcGuide(r_y, r_x) = idx_shift;
			gcGuideMask(r_y, r_x) = 1;
			numRecBB++;
		}
	}
}

void ImageViewer::estimationCooC(){

	// Initilize the confliction matrix
	int numRecBB = 0;
	vector<Point2i> idx2bb;

	for (int i_rep = 0; i_rep < bbitem.size(); i_rep++)
	{
		for (int j_rep = 0; j_rep < bbitem[i_rep].size(); j_rep++)
		{
			idx2bb.push_back(Point2i(i_rep, j_rep));
			numRecBB++;
		}
	}


	//idx2bb.resize(numRecBB);
	// compute conflict between all pairs of bb
	M_conflict = Mat1d::zeros(numRecBB, numRecBB);
	M_conflict = M_conflict - 1;
	for (int i_bb = 0; i_bb < numRecBB; i_bb++)
	{
		for (int j_bb = i_bb + 1; j_bb < numRecBB; j_bb++)
		{
			// compute the translation vector between 
			// bbitem[idx2bb[i_bb].x][idx2bb[i_bb].y] and bbitem[idx2bb[j_bb].x][idx2bb[j_bb].y]
			// compute the conflict potential using coocX_scaled[idx2bb[i_bb].x * numRep + idx2bb[j_bb].x], which stores the relative location of building block 
			// from  building block type idx2bb[j_bb].x to idx2bb[i_bb].x

			//shift_record[idx].x = i_step;
			//shift_record[idx].y = j_step;
			int i_rep_a = idx2bb[i_bb].x;
			int j_rep_a = idx2bb[i_bb].y;
			int i_rep_b = idx2bb[j_bb].x;
			int j_rep_b = idx2bb[j_bb].y;
			int X_a = bbitem[i_rep_a][j_rep_a]->x_scaled + bbitem[i_rep_a][j_rep_a]->x_shift;
			int Y_a = bbitem[i_rep_a][j_rep_a]->y_scaled + bbitem[i_rep_a][j_rep_a]->y_shift;
			int X_b = bbitem[i_rep_b][j_rep_b]->x_scaled + bbitem[i_rep_b][j_rep_b]->x_shift;
			int Y_b = bbitem[i_rep_b][j_rep_b]->y_scaled + bbitem[i_rep_b][j_rep_b]->y_shift;
			Point2i v = Point2i(X_b - X_a, Y_b - Y_a);

			if (sqrt((double)(v.x * v.x + v.y * v.y)) > cur_cooc_dist_bb)
			{

			} 
			else
			{
				int cooctype = idx2bb[i_bb].x * rep->numRep + idx2bb[j_bb].x;
				double min_dist = 100000000;
				int num_cooc = rep->coocX_scaled[cooctype].size();
				for (int i_c = 0; i_c < num_cooc; i_c++)
				{
					double dx = v.x - rep->coocX_scaled[cooctype][i_c];
					double dy = v.y - rep->coocY_scaled[cooctype][i_c];
					double temp_dist = sqrt(dx * dx + dy * dy);
					if (temp_dist == 0)
					{
						min_dist = 0;
						break;
					} 
					else
					{
						min_dist = min(min_dist, temp_dist);
					}
				}
				//min_dist = min(10, min_dist);
				M_conflict(i_bb, j_bb) = min_dist;
				M_conflict(j_bb, i_bb) = min_dist;
			}
		}
	}

	// sum up and average the conflicts
	int width = M_conflict.cols;
	double* data = (double*)M_conflict.data;
	vector<double> M_conflict_summed;
	for (int i=0;i<M_conflict.cols;i++)
	{
		double num_knn = 0;
		double column_sum=0;
		for (int k=0;k<M_conflict.rows;k++)
		{
			if (data[i + k*width] >= 0)
			{
				column_sum += data[i + k*width];
				num_knn+=1;
			}
		}
		if (num_knn == 0)
		{
			column_sum = -1;
		} 
		else
		{
			column_sum = column_sum/num_knn;
		}

		M_conflict_summed.push_back(column_sum);
	}

	//M_conflict_summed 
	//for (int i = 0; i < M_conflict_summed.size(); i++)
	//{
	//	qDebug()<<M_conflict_summed[i];
	//}

	// find the current most conflicting bb, and add it to the list_remove;
	vector<int> list_remove;
	auto maxc = max_element(M_conflict_summed.begin(), M_conflict_summed.end());
	int maxi = distance(M_conflict_summed.begin(), maxc);
	double dmaxc = M_conflict_summed[maxi];
	int rd = 0;
	while (dmaxc > cur_cooc_dist)
		//for (int i_rd = 0; i_rd < 7; i_rd++)
	{				
		// recompute M_conflict_summed
		vector<double>().swap(M_conflict_summed);
		data = (double*)M_conflict.data;
		for (int i=0;i<M_conflict.cols;i++)
		{
			double num_knn = 0;
			double column_sum=0;
			for (int k=0;k<M_conflict.rows;k++)
			{
				if (data[i + k*width] >= 0)
				{
					column_sum += data[i + k*width];
					num_knn+=1;
				}
			}
			if (num_knn == 0)
			{
				column_sum = -1;
			} 
			else
			{
				column_sum = column_sum/num_knn;
			}

			M_conflict_summed.push_back(column_sum);
		}

		// recompute the most conflicting bb
		maxc = max_element(M_conflict_summed.begin(), M_conflict_summed.end());
		maxi = distance(M_conflict_summed.begin(), maxc);
		dmaxc = M_conflict_summed[maxi];
		list_remove.push_back(maxi);
		// set all entries in M_conflict that are related to maxi to -1 
		for (int i = 0; i < M_conflict.cols; i++)
		{
			M_conflict(maxi, i) = -1;
			M_conflict(i, maxi) = -1;
		}
		rd++;
		//qDebug()<<"This is round: "<<rd<<", maxi: "<<maxi<<", dmaxc: "<<dmaxc;

	}

	qDebug()<<"M_conflict_summed: ";
	for (int i = 0; i < M_conflict_summed.size(); i++)
	{
		qDebug()<<M_conflict_summed[i];
	}


	////add all bb that has conflict value that is larger than max_cooc_dist to the removal list
	qDebug()<<"Remove list before sort: ";
	for (int i = 0; i < list_remove.size(); i++)
	{
		qDebug()<<"bb: "<<list_remove[i]<<", conflict summed: "<<M_conflict_summed[list_remove[i]]<<", x: "<<bbitem[idx2bb[list_remove[i]].x][idx2bb[list_remove[i]].y]->x_scaled + bbitem[idx2bb[list_remove[i]].x][idx2bb[list_remove[i]].y]->x_shift<<", y: "<<bbitem[idx2bb[list_remove[i]].x][idx2bb[list_remove[i]].y]->y_scaled + bbitem[idx2bb[list_remove[i]].x][idx2bb[list_remove[i]].y]->y_shift;
	}

	//-------------------------------------------------------------
	// IMPORTANT, WE NEED TO SORT the list to prevent mis-removal 
	// need to sort bb in list_remove for being correctly removed from rec_bb_list
	//-------------------------------------------------------------
	vector<vector<pair<int, int>>> sorted_list_remove;
	sorted_list_remove.resize(rep->rec_bb_list.size());
	for (int i_bb = 0; i_bb < list_remove.size(); i_bb++)
	{
		int i_rep = idx2bb[list_remove[i_bb]].x;
		int j_rep = idx2bb[list_remove[i_bb]].y;
		int ii_rep = bbitem[i_rep][j_rep]->idx2rec_bb_list.x;
		int jj_rep = bbitem[i_rep][j_rep]->idx2rec_bb_list.y;
		sorted_list_remove[ii_rep].push_back(pair<int, int>(jj_rep, list_remove[i_bb]));
		//qDebug()<<i_rep<<j_rep<<ii_rep<<jj_rep<<list_remove[i_bb]<<rep->rec_bb_list[ii_rep][jj_rep].first<<rep->rec_bb_list[ii_rep][jj_rep].second;
		//bbitem[i_rep][j_rep]->setPen(pen);
		//bbitem[i_rep][j_rep]->setBrush( QColor(255, 0, 0));
		//bbitem[i_rep][j_rep]->setZValue(qrand()%256);
		//bbitem[i_rep][j_rep]->setOpacity(0.75);
	}
	//for (int i_s = 0; i_s < rep->rec_shift_list.size(); i_s++)
	//{
	//	qDebug()<<rep->list_shiftXY_scaled[rep->rec_shift_list[i_s]].x<<rep->list_shiftXY_scaled[rep->rec_shift_list[i_s]].y;
	//}

	for (int i_s = 0; i_s < sorted_list_remove.size(); i_s++)
	{
		if (sorted_list_remove[i_s].size() > 1)
		{
			// sort sorted_list_remove[i_s]
			std::sort(sorted_list_remove[i_s].begin(),sorted_list_remove[i_s].end(), sort_pred());
		}
	}

	// put the sorted result together
	vector<int>().swap(list_remove);
	for (int i_s = 0; i_s < sorted_list_remove.size(); i_s++)
	{
		if (sorted_list_remove[i_s].size() > 0)
		{
			for (int i_bb = 0; i_bb < sorted_list_remove[i_s].size(); i_bb++)
			{
				list_remove.push_back(sorted_list_remove[i_s][i_bb].second);
			}

		}
	}

	qDebug()<<"after sort";
	for (int i = 0; i < list_remove.size(); i++)
	{
		qDebug()<<list_remove[i];
	}

	//-------------------------------------------------------------
	// update positive and negative lists
	//-------------------------------------------------------------
	vector<int>().swap(rep->neg_shift_list);
	vector<vector<pair<int, int>>>().swap(rep->neg_bb_list);
	rep->neg_shift_list.resize(rep->rec_shift_list.size());
	for (int i_s = 0; i_s < rep->rec_shift_list.size(); i_s++)
	{
		rep->neg_shift_list[i_s] = rep->rec_shift_list[i_s];
	}
	rep->neg_bb_list.resize(rep->rec_bb_list.size());

	for (int i_bb = 0; i_bb < list_remove.size(); i_bb++)
	{
		int i_rep = idx2bb[list_remove[i_bb]].x;
		int j_rep = idx2bb[list_remove[i_bb]].y;
		//remove bbitem[i_rep][j_rep] from rep->rec_bb_list
		int ii_rep = bbitem[i_rep][j_rep]->idx2rec_bb_list.x;
		int jj_rep = bbitem[i_rep][j_rep]->idx2rec_bb_list.y;
		qDebug()<<"remove "<<ii_rep<<jj_rep;
		rep->neg_bb_list[ii_rep].push_back(pair<int, int>(rep->rec_bb_list[ii_rep][jj_rep].first, rep->rec_bb_list[ii_rep][jj_rep].second));
		rep->rec_bb_list[ii_rep][jj_rep] = rep->rec_bb_list[ii_rep].back();
		rep->rec_bb_list[ii_rep].pop_back();
	}
	//-------------------------------------------------------------
	// generate positive and negative bb
	//-------------------------------------------------------------
	vector<vector<BBItem*>>().swap(bbitemPos);
	bbitemPos.resize(rep->numRep);
	for (int i_s = 0; i_s < rep->rec_bb_list.size(); i_s++)
	{
		for (int i_item = 0; i_item < rep->rec_bb_list[i_s].size(); i_item++)
		{
			int i_rep = rep->rec_bb_list[i_s][i_item].first;
			int j_rep = rep->rec_bb_list[i_s][i_item].second;
			int X = rep->repX_fullres[i_rep][j_rep] + rep->list_shiftXY_fullres[rep->rec_shift_list[i_s]].x;
			int Y = rep->repY_fullres[i_rep][j_rep] + rep->list_shiftXY_fullres[rep->rec_shift_list[i_s]].y;
			int W = rep->repW_fullres[i_rep][j_rep];
			int H = rep->repH_fullres[i_rep][j_rep];

			QPolygon polygon;
			polygon << QPoint(X, Y)
				<< QPoint(X, Y + H)
				<< QPoint(X + W, Y + H)
				<< QPoint(X + W, Y);
			BBItem *item1 = new BBItem(i_s, i_item);
			item1->idx_s = rep->rec_shift_list[i_s];
			item1->setPolygon(polygon);
			item1->x_start = 0;
			item1->y_start = 0;
			item1->x_shift = rep->list_shiftXY_scaled[rep->rec_shift_list[i_s]].x;
			item1->y_shift = rep->list_shiftXY_scaled[rep->rec_shift_list[i_s]].y;
			item1->x_shift_ori = rep->list_shiftXY_scaled[rep->rec_shift_list[i_s]].x;
			item1->y_shift_ori = rep->list_shiftXY_scaled[rep->rec_shift_list[i_s]].y;
			item1->bb_type = i_rep;
			item1->bb_idx = j_rep;
			item1->x_fullres = rep->repX_fullres[i_rep][j_rep];
			item1->y_fullres = rep->repY_fullres[i_rep][j_rep];
			item1->w_fullres = rep->repW_fullres[i_rep][j_rep];
			item1->h_fullres = rep->repH_fullres[i_rep][j_rep];
			item1->x_scaled = rep->repX_scaled[i_rep][j_rep];
			item1->y_scaled = rep->repY_scaled[i_rep][j_rep];
			item1->w_scaled = rep->repW_scaled[i_rep][j_rep];
			item1->h_scaled = rep->repH_scaled[i_rep][j_rep];
			bbitemPos[i_rep].push_back(item1);
			int cur_idx = bbitemPos[i_rep].size() - 1;
			bbitemPos[i_rep][cur_idx]->setPen(pen);
			bbitemPos[i_rep][cur_idx]->setBrush( QColor(colorList[i_rep][0], colorList[i_rep][1], colorList[i_rep][2]) );
			bbitemPos[i_rep][cur_idx]->setZValue(qrand()%256);
			bbitemPos[i_rep][cur_idx]->setOpacity(0.75);
		}
	}

	vector<vector<BBItem*>>().swap(bbitemNeg);
	bbitemNeg.resize(rep->numRep);
	for (int i_s = 0; i_s < rep->neg_bb_list.size(); i_s++)
	{
		for (int i_item = 0; i_item < rep->neg_bb_list[i_s].size(); i_item++)
		{
			int i_rep = rep->neg_bb_list[i_s][i_item].first;
			int j_rep = rep->neg_bb_list[i_s][i_item].second;
			int X = rep->repX_fullres[i_rep][j_rep] + rep->list_shiftXY_fullres[rep->neg_shift_list[i_s]].x;
			int Y = rep->repY_fullres[i_rep][j_rep] + rep->list_shiftXY_fullres[rep->neg_shift_list[i_s]].y;
			int W = rep->repW_fullres[i_rep][j_rep];
			int H = rep->repH_fullres[i_rep][j_rep];

			QPolygon polygon;
			polygon << QPoint(X, Y)
				<< QPoint(X, Y + H)
				<< QPoint(X + W, Y + H)
				<< QPoint(X + W, Y);
			BBItem *item1 = new BBItem(i_s, i_item);
			item1->setPolygon(polygon);
			item1->idx_s = rep->neg_shift_list[i_s];
			item1->x_start = 0;
			item1->y_start = 0;
			item1->x_shift = rep->list_shiftXY_scaled[rep->neg_shift_list[i_s]].x;
			item1->y_shift = rep->list_shiftXY_scaled[rep->neg_shift_list[i_s]].y;
			item1->x_shift_ori = rep->list_shiftXY_scaled[rep->neg_shift_list[i_s]].x;
			item1->y_shift_ori = rep->list_shiftXY_scaled[rep->neg_shift_list[i_s]].y;
			item1->bb_type = i_rep;
			item1->bb_idx = j_rep;
			item1->x_fullres = rep->repX_fullres[i_rep][j_rep];
			item1->y_fullres = rep->repY_fullres[i_rep][j_rep];
			item1->w_fullres = rep->repW_fullres[i_rep][j_rep];
			item1->h_fullres = rep->repH_fullres[i_rep][j_rep];
			item1->x_scaled = rep->repX_scaled[i_rep][j_rep];
			item1->y_scaled = rep->repY_scaled[i_rep][j_rep];
			item1->w_scaled = rep->repW_scaled[i_rep][j_rep];
			item1->h_scaled = rep->repH_scaled[i_rep][j_rep];
			bbitemNeg[i_rep].push_back(item1);
			int cur_idx = bbitemNeg[i_rep].size() - 1;
			bbitemNeg[i_rep][cur_idx]->setPen(pen);
			bbitemNeg[i_rep][cur_idx]->setBrush( QColor(colorList[i_rep][0], colorList[i_rep][1], colorList[i_rep][2]) );
			bbitemNeg[i_rep][cur_idx]->setZValue(qrand()%256);
			bbitemNeg[i_rep][cur_idx]->setOpacity(0.75);
		}
	}


	// update positive guidance map

	for (int i_rep = 0; i_rep < bbitemPos.size(); i_rep++)
	{
		for (int j_rep = 0; j_rep < bbitemPos[i_rep].size(); j_rep++)
		{
			//update gcLabel, gcLabelinterX, gcLabelinterY and gcGuide
			Range r_y = Range(rep->repY_scaled[bbitemPos[i_rep][j_rep]->bb_type][bbitemPos[i_rep][j_rep]->bb_idx] + bbitemPos[i_rep][j_rep]->y_shift, rep->repY_scaled[bbitemPos[i_rep][j_rep]->bb_type][bbitemPos[i_rep][j_rep]->bb_idx] + rep->repH_scaled[bbitemPos[i_rep][j_rep]->bb_type][bbitemPos[i_rep][j_rep]->bb_idx] + bbitemPos[i_rep][j_rep]->y_shift);
			Range r_x = Range(rep->repX_scaled[bbitemPos[i_rep][j_rep]->bb_type][bbitemPos[i_rep][j_rep]->bb_idx] + bbitemPos[i_rep][j_rep]->x_shift, rep->repX_scaled[bbitemPos[i_rep][j_rep]->bb_type][bbitemPos[i_rep][j_rep]->bb_idx] + rep->repW_scaled[bbitemPos[i_rep][j_rep]->bb_type][bbitemPos[i_rep][j_rep]->bb_idx] + bbitemPos[i_rep][j_rep]->x_shift);	
			r_y.start = min(rep->rowsSyn_scaled - 1, max(0, r_y.start));
			r_x.start = min(rep->colsSyn_scaled - 1, max(0, r_x.start));
			r_y.end = min(rep->rowsSyn_scaled - 1, max(0, r_y.end));
			r_x.end = min(rep->colsSyn_scaled - 1, max(0, r_x.end));
			gcPosGuide(r_y, r_x) = i_rep + 1;
		}
	}

	// update negative guidance map. neg_shift_list should have the same size as rec_shift_list
	for (int i_rep = 0; i_rep < bbitemNeg.size(); i_rep++)
	{
		for (int j_rep = 0; j_rep < bbitemNeg[i_rep].size(); j_rep++)
		{
			//update gcLabel, gcLabelinterX, gcLabelinterY and gcGuide
			Range r_y = Range(rep->repY_scaled[bbitemNeg[i_rep][j_rep]->bb_type][bbitemNeg[i_rep][j_rep]->bb_idx] + bbitemNeg[i_rep][j_rep]->y_shift, rep->repY_scaled[bbitemNeg[i_rep][j_rep]->bb_type][bbitemNeg[i_rep][j_rep]->bb_idx] + rep->repH_scaled[bbitemNeg[i_rep][j_rep]->bb_type][bbitemNeg[i_rep][j_rep]->bb_idx] + bbitemNeg[i_rep][j_rep]->y_shift);
			Range r_x = Range(rep->repX_scaled[bbitemNeg[i_rep][j_rep]->bb_type][bbitemNeg[i_rep][j_rep]->bb_idx] + bbitemNeg[i_rep][j_rep]->x_shift, rep->repX_scaled[bbitemNeg[i_rep][j_rep]->bb_type][bbitemNeg[i_rep][j_rep]->bb_idx] + rep->repW_scaled[bbitemNeg[i_rep][j_rep]->bb_type][bbitemNeg[i_rep][j_rep]->bb_idx] + bbitemNeg[i_rep][j_rep]->x_shift);	
			r_y.start = min(rep->rowsSyn_scaled - 1, max(0, r_y.start));
			r_x.start = min(rep->colsSyn_scaled - 1, max(0, r_x.start));
			r_y.end = min(rep->rowsSyn_scaled - 1, max(0, r_y.end));
			r_x.end = min(rep->colsSyn_scaled - 1, max(0, r_x.end));
			gcNegGuide(r_y, r_x) = bbitemNeg[i_rep][j_rep]->idx_s;
			//gcNegGuide(r_y, r_x) = i_rep + 1;
		}
	}
}

void ImageViewer::estimationElastic(){
	// for every bb in bbitemPos, collecting the direction for local movement
	int numPosBB = 0;
	vector<Point2i> idx2bb;
	for (int i_rep = 0; i_rep < bbitemPos.size(); i_rep++)
	{
		for (int j_rep = 0; j_rep < bbitemPos[i_rep].size(); j_rep++)
		{
			idx2bb.push_back(Point2i(i_rep, j_rep));
			numPosBB++;
		}
	}
	M_vx = Mat1d::zeros(numPosBB, numPosBB);
	M_vy = Mat1d::zeros(numPosBB, numPosBB);
	M_vx = M_vx - 1000;
	M_vy = M_vy - 1000;

	//// compute local movement for each pair of bb
	//M_conflict = Mat1d::zeros(numRecBB, numRecBB);
	for (int i_bb = 0; i_bb < numPosBB; i_bb++)
	{
		for (int j_bb = i_bb + 1; j_bb < numPosBB; j_bb++)
		{
			//for (int i_bb = 0; i_bb < 10; i_bb++)
			//{
			//	for (int j_bb = i_bb + 1; j_bb < numPosBB; j_bb++)
			//	{
			// compute the translation vector between 
			// bbitem[idx2bb[i_bb].x][idx2bb[i_bb].y] and bbitem[idx2bb[j_bb].x][idx2bb[j_bb].y]
			// compute the conflict potential using coocX_scaled[idx2bb[i_bb].x * numRep + idx2bb[j_bb].x], which stores the relative location of building block 
			// from  building block type idx2bb[j_bb].x to idx2bb[i_bb].x

			int i_rep_a = idx2bb[i_bb].x;
			int j_rep_a = idx2bb[i_bb].y;
			int i_rep_b = idx2bb[j_bb].x;
			int j_rep_b = idx2bb[j_bb].y;
			int X_a = bbitemPos[i_rep_a][j_rep_a]->x_scaled + bbitemPos[i_rep_a][j_rep_a]->x_shift;
			int Y_a = bbitemPos[i_rep_a][j_rep_a]->y_scaled + bbitemPos[i_rep_a][j_rep_a]->y_shift;
			int X_b = bbitemPos[i_rep_b][j_rep_b]->x_scaled + bbitemPos[i_rep_b][j_rep_b]->x_shift;
			int Y_b = bbitemPos[i_rep_b][j_rep_b]->y_scaled + bbitemPos[i_rep_b][j_rep_b]->y_shift;
			Point2i v = Point2i(X_b - X_a, Y_b - Y_a);

			if (sqrt((double)(v.x * v.x + v.y * v.y)) > cur_cooc_dist_bb)
			{

			} 
			else
			{
				int cooctype = idx2bb[i_bb].x * rep->numRep + idx2bb[j_bb].x;
				int num_cooc = rep->coocX_scaled[cooctype].size();
				double min_dist = 100000000;
				double min_vx = 0;
				double min_vy = 0;

				for (int i_c = 0; i_c < num_cooc; i_c++)
				{	
					double dx = double(-v.x) + double(rep->coocX_scaled[cooctype][i_c]);
					double dy = double(-v.y) + double(rep->coocY_scaled[cooctype][i_c]);
					double temp_dist = sqrt(dx * dx + dy * dy);
					if (temp_dist == 0)
					{
						min_dist = 0;
						min_vx = 0;
						min_vy = 0;
						break;
					} 
					else
					{
						min_dist = min(min_dist, temp_dist);
						if (min_dist == temp_dist)
						{
							min_vx = -dx;
							min_vy = -dy;
						}
					}
				}
				qDebug()<<"cooctype: "<<cooctype<<"min_dist: "<<min_dist<<", min_vx: "<<min_vx<<", min_vy: "<<min_vy;
				M_vx(i_bb, j_bb) = min_vx;
				M_vx(j_bb, i_bb) = min_vx;
				M_vy(i_bb, j_bb) = min_vy;
				M_vy(j_bb, i_bb) = min_vy;
			}
		}
	}

	// 
	//for (int i = 0; i < 1; i++)
	//{
	//	for (int j = 0; j < M_vx.cols; j++)
	//	{
	//		qDebug()<<M_vx(i, j);
	//	}
	//}

	// sum up and average the conflicts
	int width = M_vx.cols;
	double* data_vx = (double*)M_vx.data;
	double* data_vy = (double*)M_vy.data;

	vector<double> averaged_vx;
	vector<double> averaged_vy;

	for (int i=0;i<M_vx.cols;i++)
	{
		double num_knn = 0;
		double column_sum=0;
		for (int k=0;k<M_vx.rows;k++)
		{
			if (data_vx[i + k*width] != -1000)
			{
				column_sum += data_vx[i + k*width];
				num_knn+=1;
				//if (i == 0)
				//{
				//	qDebug()<<data_vx[i + k*width];
				//}
			}

		}
		if (num_knn == 0)
		{
			column_sum = 100;
		} 
		else
		{
			//column_sum = round(column_sum/num_knn);
			column_sum = (column_sum/num_knn);
		}

		averaged_vx.push_back(column_sum);
	}

	for (int i=0;i<M_vy.cols;i++)
	{
		double num_knn = 0;
		double column_sum=0;
		for (int k=0;k<M_vy.rows;k++)
		{
			if (data_vy[i + k*width] != -1000)
			{
				column_sum += data_vy[i + k*width];
				num_knn+=1;
			}

		}
		if (num_knn == 0)
		{
			column_sum = 100;
		} 
		else
		{
			/*column_sum = round(column_sum/num_knn);*/
			column_sum = (column_sum/num_knn);
		}

		averaged_vy.push_back(column_sum);
	}

	//for (int i = 0; i < averaged_vx.size(); i++)
	//{
	//	qDebug()<<"averaged_vx: "<<averaged_vx[i]<<", averaged_vy: "<<averaged_vy[i];
	//}

	// update numPosBB 
	for (int i_item = 0; i_item < averaged_vx.size(); i_item++)
	{
		int i_rep = idx2bb[i_item].x;
		int j_rep = idx2bb[i_item].y;
		//bbitemPos[i_rep][j_rep]->x_shift += 1;
		//bbitemPos[i_rep][j_rep]->y_shift += 1;
		//if (i_item < 10)
		//{
		bbitemPos[i_rep][j_rep]->x_shift += round(averaged_vx[i_item]);
		bbitemPos[i_rep][j_rep]->y_shift += round(averaged_vy[i_item]);
		qDebug()<<"averaged_vx: "<<round(averaged_vx[i_item])<<", averaged_vy: "<<round(averaged_vy[i_item]);
		QPolygon polygon;
		polygon << QPoint(bbitemPos[i_rep][j_rep]->x_fullres + round(bbitemPos[i_rep][j_rep]->x_shift/scalerRes), bbitemPos[i_rep][j_rep]->y_fullres + round(bbitemPos[i_rep][j_rep]->y_shift/scalerRes))
			<< QPoint(bbitemPos[i_rep][j_rep]->x_fullres + round(bbitemPos[i_rep][j_rep]->x_shift/scalerRes), bbitemPos[i_rep][j_rep]->y_fullres + round(bbitemPos[i_rep][j_rep]->y_shift/scalerRes) + bbitemPos[i_rep][j_rep]->h_fullres)
			<< QPoint(bbitemPos[i_rep][j_rep]->x_fullres + round(bbitemPos[i_rep][j_rep]->x_shift/scalerRes) + bbitemPos[i_rep][j_rep]->w_fullres, bbitemPos[i_rep][j_rep]->y_fullres + round(bbitemPos[i_rep][j_rep]->y_shift/scalerRes) + bbitemPos[i_rep][j_rep]->h_fullres)
			<< QPoint(bbitemPos[i_rep][j_rep]->x_fullres + round(bbitemPos[i_rep][j_rep]->x_shift/scalerRes) + bbitemPos[i_rep][j_rep]->w_fullres, bbitemPos[i_rep][j_rep]->y_fullres + round(bbitemPos[i_rep][j_rep]->y_shift/scalerRes));
		bbitemPos[i_rep][j_rep]->setPolygon(polygon);
		//}
	}


	//// update positive guidance map
	//for (int i_rep = 0; i_rep < bbitemPos.size(); i_rep++)
	//{
	//	for (int j_rep = 0; j_rep < bbitemPos[i_rep].size(); j_rep++)
	//	{
	//		//update gcLabel, gcLabelinterX, gcLabelinterY and gcGuide
	//		Range r_y = Range(rep->repY_scaled[bbitemPos[i_rep][j_rep]->bb_type][bbitemPos[i_rep][j_rep]->bb_idx] + bbitemPos[i_rep][j_rep]->y_shift, rep->repY_scaled[bbitemPos[i_rep][j_rep]->bb_type][bbitemPos[i_rep][j_rep]->bb_idx] + rep->repH_scaled[bbitemPos[i_rep][j_rep]->bb_type][bbitemPos[i_rep][j_rep]->bb_idx] + bbitemPos[i_rep][j_rep]->y_shift);
	//		Range r_x = Range(rep->repX_scaled[bbitemPos[i_rep][j_rep]->bb_type][bbitemPos[i_rep][j_rep]->bb_idx] + bbitemPos[i_rep][j_rep]->x_shift, rep->repX_scaled[bbitemPos[i_rep][j_rep]->bb_type][bbitemPos[i_rep][j_rep]->bb_idx] + rep->repW_scaled[bbitemPos[i_rep][j_rep]->bb_type][bbitemPos[i_rep][j_rep]->bb_idx] + bbitemPos[i_rep][j_rep]->x_shift);	
	//		r_y.start = min(rep->rowsSyn_scaled - 1, max(0, r_y.start));
	//		r_x.start = min(rep->colsSyn_scaled - 1, max(0, r_x.start));
	//		r_y.end = min(rep->rowsSyn_scaled - 1, max(0, r_y.end));
	//		r_x.end = min(rep->colsSyn_scaled - 1, max(0, r_x.end));
	//		gcPosGuide(r_y, r_x) = i_rep + 1;
	//	}
	//}
}

void ImageViewer::setGlobalConstraint(){
	// for each bb, set up a global constraint
	// boundary constraint
	// 1 means left boundary, 
	// 2 means right boundary,
	// 3 means upper boundary 
	// 4 means lower boundary
	//list_boundary_constraint.resize(3);
	//list_boundary_constraint[1].resize(1);
	//list_boundary_constraint[1][0] = 4;
	//list_boundary_constraint[2].resize(2);
	//list_boundary_constraint[2][0] = 1;
	//list_boundary_constraint[2][0] = 2;

	// set up a valid boundary regions for each bb
	regioninvalid_boundary_constraint.resize(3);
	regioninvalid_boundary_constraint[0].resize(4);
	regioninvalid_boundary_constraint[0][0] = -1; // start x
	regioninvalid_boundary_constraint[0][1] = -1; // start y
	regioninvalid_boundary_constraint[0][2] = -1;     // end x
	regioninvalid_boundary_constraint[0][3] = -1; // end y
	regioninvalid_boundary_constraint[1].resize(4);
	regioninvalid_boundary_constraint[1][0] = 0; // start x
	regioninvalid_boundary_constraint[1][1] = 0; // start y
	regioninvalid_boundary_constraint[1][2] = rep->colsSyn_scaled;     // end x
	regioninvalid_boundary_constraint[1][3] = rep->rowsSyn_scaled - 12; // end y
	regioninvalid_boundary_constraint[2].resize(4);
	//regioninvalid_boundary_constraint[2][0] = -1; // start x
	//regioninvalid_boundary_constraint[2][1] = -1; // start y
	//regioninvalid_boundary_constraint[2][2] = -1;     // end x
	//regioninvalid_boundary_constraint[2][3] = -1; // end y
	regioninvalid_boundary_constraint[2][0] = 12; // start x
	regioninvalid_boundary_constraint[2][1] = 0; // start y
	regioninvalid_boundary_constraint[2][2] = rep->colsSyn_scaled - 12;     // end x
	regioninvalid_boundary_constraint[2][3] = rep->rowsSyn_scaled - 1; // end y
}

void ImageViewer::maximizationSyn(){
	synmode = 4;
	//synFree();
	if (flag_MS)
	{
		synSampleMSL();
	} 
	else
	{
		synSample();
	}

}

void ImageViewer::synOpt(){
	if (rep->num_shiftX > 1 || rep->num_shiftY > 1)
	{
		maximizationSyn();
	}
	

	/*
	qDebug()<<"*************************************************************************************** ";
	qDebug()<<"Co-occurrence optimization starts: ... ";
	qDebug()<<"*************************************************************************************** ";

	//gcPosGuide = Mat1b::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled);
	//gcPosGuide = gcPosGuide - 1;
	//gcNegGuide = Mat1b::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled);
	//gcNegGuide = gcNegGuide - 1;
	for (i_iter_em = 0; i_iter_em < max_iter_em; i_iter_em++)
	{
		//cur_cooc_dist = cur_cooc_dist * 0.95;
		cur_cooc_dist_bb = max(round(rep->rowsSyn_scaled * cur_cooc_dist_ratio), round(rep->colsSyn_scaled * cur_cooc_dist_ratio));
		gcPosGuide = Mat1b::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled);
		gcPosGuide = gcPosGuide - 1;
		gcNegGuide = Mat1b::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled);
		gcNegGuide = gcNegGuide - 1;
		//------------------------------------------------------------
		// Estimation
		//------------------------------------------------------------
		//----------------------------------------------------
		// first sub step: sliding window detection of building blocks
		//----------------------------------------------------
		estimationSW();
		renderRepBB();
		QString filename_img_estimationSW = "C:/Chuan/Dropbox/Project/2DBuildingBlock/SiggraphAsia/Code/ImageSyn/imageviewer/result/img_";
		filename_img_estimationSW.append(QString ("%1").arg(i_iter_em + 1));		
		filename_img_estimationSW.append("_estimation_1_SW");
		filename_img_estimationSW.append(".jpg");
		QImage img_estimationSW(rep->qimgSyn_fullres->width(), rep->qimgSyn_fullres->height(), QImage::Format_RGB32);
		QPainter p_estimationSW(&img_estimationSW);
		scene->render(&p_estimationSW);
		p_estimationSW.end();
		img_estimationSW.save(filename_img_estimationSW);

		estimationCooC();
		renderRepBB();
		QString filename_img_estimationCooC = "C:/Chuan/Dropbox/Project/2DBuildingBlock/SiggraphAsia/Code/ImageSyn/imageviewer/result/img_";
		filename_img_estimationCooC.append(QString ("%1").arg(i_iter_em + 1));
		filename_img_estimationCooC.append("_estimation_2_CooC");		
		filename_img_estimationCooC.append(".jpg");
		QImage img_estimationCooC(rep->qimgSyn_fullres->width(), rep->qimgSyn_fullres->height(), QImage::Format_RGB32);
		QPainter p_estimationCooC(&img_estimationCooC);
		scene->render(&p_estimationCooC);
		p_estimationCooC.end();
		img_estimationCooC.save(filename_img_estimationCooC);

		//------------------------------------------------------------
		// Maximization
		//------------------------------------------------------------
		maximizationSyn();
		//------------------------------------------------------------
		// OPTIONAL: Make the estimation more accurate by increasing cur_cooc_dist and cur_cooc_dist_bb
		//------------------------------------------------------------
		qDebug()<<"cur_cooc_dist: "<<cur_cooc_dist<<", cur_cooc_dist_bb: "<<cur_cooc_dist_bb;
	}

	//maximizationSyn();
	imgDisp = new QGraphicsPixmapItem(QPixmap::fromImage(*rep->qimgSyn_fullres));
	scene->clear();
	scene->addItem(imgDisp);
	scene->setSceneRect(0, 0, rep->qimgSyn_fullres->width(), rep->qimgSyn_fullres->height());
	view = new QGraphicsView(scene);
	setCentralWidget(view);
	resize(rep->qimgSyn_fullres->width() + 10, rep->qimgSyn_fullres->height() + 50);	
	qDebug()<<"Co-occurrence optimization finished.";  

	//renderInput();
	*/
}

void ImageViewer::renderInput(){	
	flag_sideselection = true;
	newscene = new QGraphicsScene();
	QImage* qimgtemp = new QImage(filename_imgInput);
	QGraphicsPixmapItem* imgDispInput = new QGraphicsPixmapItem(QPixmap::fromImage(*qimgtemp));
	newscene->addItem(imgDispInput);
	newscene->setSceneRect(0, 0, rep->qimgInput_fullres->width(), rep->qimgInput_fullres->height());
	newview = new QGraphicsView(newscene);
	newviewer = new sideviewer;
	newviewer->setCentralWidget(newview);
	newviewer->show();
	newviewer->resize(rep->qimgInput_fullres->width() + 10, rep->qimgInput_fullres->height() + 50);
	newviewer->setWindowTitle(tr("ImageInput"));

	// show all BB from the detection
	vector<vector<BBItem*>>().swap(bbitemInput);
	bbitemInput.resize(rep->numRep);
	for (int i_rep = 0; i_rep < rep->numRep; i_rep++)
	{
		for (int j_rep = 0; j_rep < rep->repX_fullres[i_rep].size(); j_rep++)
		{
			int X = rep->repX_fullres[i_rep][j_rep];
			int Y = rep->repY_fullres[i_rep][j_rep];
			int W = rep->repW_fullres[i_rep][j_rep];
			int H = rep->repH_fullres[i_rep][j_rep];
			QPolygon polygon;
			polygon << QPoint(X, Y)
				<< QPoint(X, Y + H)
				<< QPoint(X + W, Y + H)
				<< QPoint(X + W, Y);

			BBItem *item1 = new BBItem(0, 0);
			item1->setPolygon(polygon);
			item1->x_start = 0;
			item1->y_start = 0;
			item1->x_shift = 0;
			item1->y_shift = 0;
			item1->x_shift_ori = 0;
			item1->y_shift_ori = 0;
			item1->bb_type = i_rep;
			item1->bb_idx = j_rep;

			bbitemInput[i_rep].push_back(item1);
			int cur_idx = bbitemInput[i_rep].size() - 1;
			bbitemInput[i_rep][cur_idx]->setPen(pen);
			bbitemInput[i_rep][cur_idx]->setBrush( QColor(colorList[i_rep][0], colorList[i_rep][1], colorList[i_rep][2]) );
			bbitemInput[i_rep][cur_idx]->setZValue(qrand()%256);
			bbitemInput[i_rep][cur_idx]->setOpacity(0.75);
			bbitemInput[i_rep][cur_idx]->setFlag(QGraphicsItem::ItemIsMovable, false);
			newscene->addItem(bbitemInput[i_rep][cur_idx]);
		}
	}

	//qDebug()<<"showInput finished";
}

void ImageViewer::computeKNNbb(){
	vector<vector<Point3d>>().swap(rep->pixel2bb);
	rep->pixel2bb.resize(rep->rowsInput_scaled * rep->colsInput_scaled);
	// NOTICE THIS IS NOT THE MATLAB ORDER
	// to keep consistent with gcNodesInput
	for (int i_r = 0; i_r < rep->rowsInput_scaled; i_r++)
	{
		for (int i_c = 0; i_c < rep->colsInput_scaled; i_c++)
		{
			// compute the KNN bb for pixel (i_r, i_c)
			rep->pixel2bb[i_r * rep->colsInput_scaled + i_c].resize(cur_num_KNN);
			// compute the distance from [i_c, i_r] to all bb
			vector<pair<double, int>> dist_temp;
			// record the relative location of all bb to [i_c, i_r]
			vector<Point3d> loc_temp;
			int count = 0;
			for (int i_rep = 0; i_rep < rep->numRep; i_rep++)
			{
				for (int j_rep = 0; j_rep < rep->sizeRep[i_rep]; j_rep++)
				{
					double vx = rep->repX_scaled[i_rep][j_rep] - i_c;
					double vy = rep->repY_scaled[i_rep][j_rep] - i_r;
					loc_temp.push_back(Point3d((double)i_rep, vx, vy));
					dist_temp.push_back(pair<double, int>(sqrt(vx*vx + vy*vy), count));
					count++;
				}
			}
			// find the K nearest neighbor
			std::sort(dist_temp.begin(),dist_temp.end(), sort_double_increase());
			for (int i_k = 0; i_k < cur_num_KNN; i_k++)
			{
				int id = dist_temp[i_k].second;
				rep->pixel2bb[i_r * rep->colsInput_scaled + i_c][i_k] = Point3d(loc_temp[id].x, loc_temp[id].y, loc_temp[id].z);
			}
		}
	}
}

void ImageViewer::computeConsMt(){
	int num_node = gcNodesInput.size();
	M_constellation = Mat1d::zeros(num_node, num_node);
	for (int i_n = 0; i_n < num_node; i_n++)
	{
		for (int j_n = i_n + 1; j_n < num_node; j_n++)
		{
			// compute the constellation distance between rep->pixel2bb[i_n] and rep->pixel2bb[j_n]
			double distp2p = 0;
			for (int i_bb = 0; i_bb < cur_num_KNN; i_bb++)
			{
				double temp_dist = 100; // default maximum distance for mismatch
				// find the closet bb in rep->pixel2bb[j_n]
				for (int j_bb = 0; j_bb < cur_num_KNN; j_bb++)
				{
					 //rep->pixel2bb[i_n] rep->pixel2bb[j_n]
					if (rep->pixel2bb[i_n][i_bb].x == rep->pixel2bb[j_n][j_bb].x)
					{
						double d_vx = rep->pixel2bb[i_n][i_bb].y - rep->pixel2bb[j_n][j_bb].y;
						double d_vy = rep->pixel2bb[i_n][i_bb].z - rep->pixel2bb[j_n][j_bb].z;
						double d = sqrt((d_vx * d_vx + d_vy * d_vy));
						temp_dist = min(temp_dist, d);
					} 
					else
					{
					}
				}
				distp2p += temp_dist;
			}


			// need to perform a symmetry computation from rep->pixel2bb[j_n] to rep->pixel2bb[i_n], otherwise the submodular constraint won't be met for alpha expansion

			for (int i_bb = 0; i_bb < cur_num_KNN; i_bb++)
			{
				double temp_dist = 100; // default maximum distance for mismatch
				// find the closet bb in rep->pixel2bb[j_n]
				for (int j_bb = 0; j_bb < cur_num_KNN; j_bb++)
				{
					//rep->pixel2bb[i_n] rep->pixel2bb[j_n]
					if (rep->pixel2bb[j_n][i_bb].x == rep->pixel2bb[i_n][j_bb].x)
					{
						double d_vx = rep->pixel2bb[j_n][i_bb].y - rep->pixel2bb[i_n][j_bb].y;
						double d_vy = rep->pixel2bb[j_n][i_bb].z - rep->pixel2bb[i_n][j_bb].z;
						double d = sqrt((d_vx * d_vx + d_vy * d_vy));
						temp_dist = min(temp_dist, d);
					} 
					else
					{
					}
				}
				distp2p += temp_dist;
			}

			// put the distance into rep->M_constellation(i_n, j_n) and rep->M_constellation(j_n, i_n)
			M_constellation(i_n, j_n) = (int)round(distp2p);
			M_constellation(j_n, i_n) = M_constellation(i_n, j_n);
			max_dist_constellation = max(max_dist_constellation, distp2p);

		}
	}

	// maybe we have have a look at rep->M_constellation
	//int i_sel = 0; 
	//int j_sel = 1;
	//qDebug()<<"size of max_dist_constellation: "<<M_constellation.rows<<M_constellation.cols;
	//qDebug()<<"distance between "<<i_sel<<" and "<<j_sel<<" is : "<<M_constellation(i_sel, j_sel);
	qDebug()<<"max distance is "<<max_dist_constellation;
	//imshow("M_constellation", M_constellation/max_dist_constellation);
}

void ImageViewer::computeConsMtHungarian(){
	int num_node = gcNodesInput.size();
	M_constellation = Mat1d::zeros(num_node, num_node);
	for (int i_n = 0; i_n < num_node; i_n++)
	{
		for (int j_n = i_n + 1; j_n < num_node; j_n++)
		{
	//for (int i_n = 0; i_n < 1; i_n++)
	//{
	//	for (int j_n = i_n + 1; j_n < 2; j_n++)
	//	{
			// compute the distance matrix between rep->pixel2bb[i_n] and rep->pixel2bb[j_n]
			hungarian_problem_t p;
			int** m;
			m = (int**)calloc(cur_num_KNN,sizeof(int*));
			for (int i_bb = 0; i_bb < cur_num_KNN; i_bb++)
			{
				m[i_bb] = (int*)calloc(cur_num_KNN,sizeof(int));
				// find the closet bb in rep->pixel2bb[j_n]
				for (int j_bb = 0; j_bb < cur_num_KNN; j_bb++)
				{
					//rep->pixel2bb[i_n] rep->pixel2bb[j_n]
					if (rep->pixel2bb[i_n][i_bb].x == rep->pixel2bb[j_n][j_bb].x)
					{
						double d_vx = rep->pixel2bb[i_n][i_bb].y - rep->pixel2bb[j_n][j_bb].y;
						double d_vy = rep->pixel2bb[i_n][i_bb].z - rep->pixel2bb[j_n][j_bb].z;
						m[i_bb][j_bb] = sqrt((d_vx * d_vx + d_vy * d_vy));
					} 
					else
					{
						m[i_bb][j_bb] = 100;
					}
				}
			}

			//for (int i_bb = 0; i_bb < cur_num_KNN; i_bb++){
			//	for (int j_bb = 0; j_bb < cur_num_KNN; j_bb++){
			//	    qDebug()<<m[i_bb][j_bb];
			//	}
			//}


			int matrix_size = hungarian_init(&p, m, cur_num_KNN, cur_num_KNN, HUNGARIAN_MODE_MINIMIZE_COST) ;
			hungarian_solve(&p);

			for (int i_bb = 0; i_bb < cur_num_KNN; i_bb++){
				for (int j_bb = 0; j_bb < cur_num_KNN; j_bb++){
					if (p.assignment[i_bb][j_bb] == 1)
					{
						//qDebug()<<p.cost[i_bb][j_bb];
						M_constellation[i_n][j_n] += m[i_bb][j_bb];
						M_constellation[j_n][i_n] = M_constellation[i_n][j_n];
					}
				}
			}

			hungarian_free(&p);
			free(m);
		}
	}

	//for (int i_n = 0; i_n < num_node; i_n++)
	//{
	//	for (int j_n = i_n + 1; j_n < num_node; j_n++)
	//	{
	//		qDebug()<<M_constellation[i_n][j_n];
	//	}
	//}
}

void ImageViewer::computeConsMDS(){
	qDebug()<<"computeConsMDS starts ...";
	//-------------------------------------------------------------------------------
	// first, compute the pairwise constellation distance using Hungarian's algorithm
	//-------------------------------------------------------------------------------
	int num_node = gcNodesInput.size();
	Mat1d M_consH = Mat1d::zeros(num_node, num_node);
	for (int i_n = 0; i_n < num_node; i_n++)
	{
		for (int j_n = i_n + 1; j_n < num_node; j_n++)
		{
			// compute the distance matrix between rep->pixel2bb[i_n] and rep->pixel2bb[j_n]
			hungarian_problem_t p;
			int** m;
			m = (int**)calloc(cur_num_KNN,sizeof(int*));
			for (int i_bb = 0; i_bb < cur_num_KNN; i_bb++)
			{
				m[i_bb] = (int*)calloc(cur_num_KNN,sizeof(int));
				// find the closet bb in rep->pixel2bb[j_n]
				for (int j_bb = 0; j_bb < cur_num_KNN; j_bb++)
				{
					//rep->pixel2bb[i_n] rep->pixel2bb[j_n]
					if (rep->pixel2bb[i_n][i_bb].x == rep->pixel2bb[j_n][j_bb].x)
					{
						double d_vx = rep->pixel2bb[i_n][i_bb].y - rep->pixel2bb[j_n][j_bb].y;
						double d_vy = rep->pixel2bb[i_n][i_bb].z - rep->pixel2bb[j_n][j_bb].z;
						m[i_bb][j_bb] = sqrt((d_vx * d_vx + d_vy * d_vy));
					} 
					else
					{
						m[i_bb][j_bb] = 100;
					}
				}
			}

			int matrix_size = hungarian_init(&p, m, cur_num_KNN, cur_num_KNN, HUNGARIAN_MODE_MINIMIZE_COST) ;
			hungarian_solve(&p);

			for (int i_bb = 0; i_bb < cur_num_KNN; i_bb++){
				for (int j_bb = 0; j_bb < cur_num_KNN; j_bb++){
					if (p.assignment[i_bb][j_bb] == 1)
					{
						//qDebug()<<p.cost[i_bb][j_bb];
						M_consH[i_n][j_n] += (double)m[i_bb][j_bb];
					}
				}
			}
			M_consH[j_n][i_n] = M_consH[i_n][j_n];

			hungarian_free(&p);
			free(m);
		}
	}

	//-------------------------------------------------------------------------------
	// perform MDS 
	//-------------------------------------------------------------------------------
	int dim = min(10, num_node);
	int iter=1; // number of iterations
	smat::Matrix<double> *A=new smat::Matrix<double>(num_node, num_node, 1);
	for (int i_n = 0; i_n < num_node; i_n++)
	{
		for (int j_n = 0; j_n < num_node; j_n++)
		{
			A->set(i_n, j_n, (double)M_consH[i_n][j_n]);
		}
	}
	smat::Matrix<double> * X1 = MDS_UCF(A, NULL, dim, iter); 

	//-------------------------------------------------------------------------------
	// recompute Euclidean distance between each row in X1
	//-------------------------------------------------------------------------------
	smat::Matrix<int> *X2 = new smat::Matrix<int>(num_node, dim);
	for (int i_n = 0; i_n < num_node; i_n++)
	{
		for(int j_n = 0; j_n < dim; j_n++){
			X2->set(i_n, j_n, round(X1->get(i_n, j_n)/dim)); // divided by dimension so keep the influence at the same degree as the other term
		}
	}

	M_constellation = Mat1d::zeros(num_node, num_node);
	for (int i_n = 0; i_n < num_node; i_n++)
	{
		for (int j_n = i_n + 1; j_n < num_node; j_n++){
			for (int i = 0; i < dim; i++)
			{
				M_constellation[i_n][j_n] = M_constellation[i_n][j_n] + (double)abs(X2->get(i_n, i) - X2->get(j_n, i));
			}
			M_constellation[j_n][i_n] = M_constellation[i_n][j_n];
		}
	}

}

void ImageViewer::updateGuide(){
	synmode = 2;
	flag_interactive = true;
	flag_scribble = false;
    qDebug()<<"Here to update guidance";
	for (int i_rep = 0; i_rep < bbitem.size(); i_rep++)
	{
		if (bbitem[i_rep].size() > 0)
		{
			for (int j_rep = 0; j_rep < bbitem[i_rep].size(); j_rep++)
			{
				if (bbitem[i_rep][j_rep]->flag_changed)
				{
					//update the x_shift and y_shift for this item
					bbitem[i_rep][j_rep]->x_shift = bbitem[i_rep][j_rep]->x_shift_ori + round(bbitem[i_rep][j_rep]->x() * scalerRes);
					bbitem[i_rep][j_rep]->y_shift = bbitem[i_rep][j_rep]->y_shift_ori + round(bbitem[i_rep][j_rep]->y() * scalerRes);
				}
			}
		}
	}

	// update list of shift and gcLabels (somehow they are coupled ~~)
	//initialize rep->list_shift_scaled as 1-d grid
	rep->num_shiftX = 0;
	rep->num_shiftY = 0;
	vector<int>().swap(rep->list_shiftX_scaled);
	vector<int>().swap(rep->list_shiftY_scaled);
	vector<int>().swap(rep->list_shiftX_fullres);
	vector<int>().swap(rep->list_shiftY_fullres);
	vector<Point2i>().swap(rep->list_shiftXY_scaled);
	vector<Point2i>().swap(rep->list_shiftXY_fullres);
	vector<Point2i>().swap(gcLabels);

	gcGuide = Mat1d::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled);
	gcGuide = gcGuide - 1;
	gcGuideMask = Mat1d::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled);

	// EM... this is the important bit
	// The first entry in the list is the offset of the last edited item
	int numRecBB = 0;
	for (int i_rep = 0; i_rep < bbitem.size(); i_rep++)
	{
		for (int j_rep = 0; j_rep < bbitem[i_rep].size(); j_rep++)
		{
			if (bbitem[i_rep][j_rep]->flag_changed)
			{
				//update gcLabel, gcLabelinterX, gcLabelinterY and gcGuide
				//Range r_y = Range(rep->repY_scaled[bbitem[i_rep][j_rep]->bb_type][bbitem[i_rep][j_rep]->bb_idx], rep->repY_scaled[bbitem[i_rep][j_rep]->bb_type][bbitem[i_rep][j_rep]->bb_idx] + rep->repH_scaled[bbitem[i_rep][j_rep]->bb_type][bbitem[i_rep][j_rep]->bb_idx]);
				//Range r_x = Range(rep->repX_scaled[bbitem[i_rep][j_rep]->bb_type][bbitem[i_rep][j_rep]->bb_idx] + bbitem[i_rep][j_rep]->x_shift, rep->repX_scaled[bbitem[i_rep][j_rep]->bb_type][bbitem[i_rep][j_rep]->bb_idx] + rep->repW_scaled[bbitem[i_rep][j_rep]->bb_type][bbitem[i_rep][j_rep]->bb_idx] + bbitem[i_rep][j_rep]->x_shift);	
				//r_y.start = min(rep->rowsSyn_scaled - 1, max(0, r_y.start));
				//r_x.start = min(rep->colsSyn_scaled - 1, max(0, r_x.start));
				//r_y.end = min(rep->rowsSyn_scaled - 1, max(0, r_y.end));
				//r_x.end = min(rep->colsSyn_scaled - 1, max(0, r_x.end));

				// update gcGuide
				rep->list_shiftX_scaled.push_back(bbitem[i_rep][j_rep]->x_shift);
				rep->list_shiftY_scaled.push_back(bbitem[i_rep][j_rep]->y_shift);

				//int idx_shift = *std::find(rep->list_shiftX_scaled.begin(), rep->list_shiftX_scaled.end(), bbitem[i_rep][j_rep]->x_shift);
				//auto loc = std::find(rep->list_shiftX_scaled.begin(), rep->list_shiftX_scaled.end(), bbitem[i_rep][j_rep]->x_shift);
				//int idx_shift = loc - rep->list_shift_scaled.begin();
				//gcGuide(r_y, r_x) = idx_shift;
				//gcGuideMask(r_y, r_x) = 1;

				rep->num_shiftX++;
				rep->num_shiftY++;
			}
		}
	}	

	// then expand the remaining list using statistical sampling
	// first, compute the minimum shift that cover the whole image
	int range_shift_x = rep->colsSyn_scaled - rep->colsInput_scaled;
	int range_shift_y = rep->rowsSyn_scaled - rep->rowsInput_scaled;

	// expand in x direction ... 
	// do insertion at the front, for efficiency should use deque, but doesn't matter for now
	while(rep->list_shiftX_scaled[0] > x_shift_lowbd){
		// sample a shift
		int gen_X = rand() % rep->num_bbstatisticsX;
		rep->expansion_bbstatisticsX = rep->list_bbstatisticsX[gen_X];
		rep->expansion_bbstatisticsX_scaled = round(rep->expansion_bbstatisticsX * scalerRes);
		rep->list_shiftX_scaled.insert(rep->list_shiftX_scaled.begin(),rep->list_shiftX_scaled[0] - rep->expansion_bbstatisticsX_scaled);
		rep->num_shiftX += 1;
	}
	// do insertion at the end
	while(rep->list_shiftX_scaled[rep->list_shiftX_scaled.size() - 1] < range_shift_x + x_shift_upbd){
		int gen_X = rand() % rep->num_bbstatisticsX;
		rep->expansion_bbstatisticsX = rep->list_bbstatisticsX[gen_X];
		rep->expansion_bbstatisticsX_scaled = round(rep->expansion_bbstatisticsX * scalerRes);
		rep->list_shiftX_scaled.push_back(rep->list_shiftX_scaled[rep->list_shiftX_scaled.size() - 1] + rep->expansion_bbstatisticsX_scaled);
		rep->num_shiftX += 1;
	}
	while(rep->list_shiftY_scaled[0] > y_shift_lowbd){
		// sample a shift
		int gen_Y = rand() % rep->num_bbstatisticsY;
		rep->expansion_bbstatisticsY = rep->list_bbstatisticsY[gen_Y];
		rep->expansion_bbstatisticsY_scaled = round(rep->expansion_bbstatisticsY * scalerRes);
		rep->list_shiftY_scaled.insert(rep->list_shiftY_scaled.begin(),rep->list_shiftY_scaled[0] - rep->expansion_bbstatisticsY_scaled);
		rep->num_shiftY += 1;
	}
	// do insertion at the end
	while(rep->list_shiftY_scaled[rep->list_shiftY_scaled.size() - 1] < range_shift_y + y_shift_upbd){
		int gen_Y = rand() % rep->num_bbstatisticsY;
		rep->expansion_bbstatisticsY = rep->list_bbstatisticsY[gen_Y];
		rep->expansion_bbstatisticsY_scaled = round(rep->expansion_bbstatisticsY * scalerRes);
		rep->list_shiftY_scaled.push_back(rep->list_shiftY_scaled[rep->list_shiftY_scaled.size() - 1] + rep->expansion_bbstatisticsY_scaled);
		rep->num_shiftY += 1;
	}
	//qDebug()<<"list_shiftX_scaled: ";
	//for (int i = 0; i < rep->num_shiftX; i++)
	//{
	//	qDebug()<<rep->list_shiftX_scaled[i];
	//}

	//qDebug()<<"list_shiftY_scaled: ";
	//for (int i = 0; i < rep->num_shiftY; i++)
	//{
	//	qDebug()<<rep->list_shiftY_scaled[i];
	//}
	//// compute the candidate shift
	rep->num_shiftXY = rep->num_shiftX * rep->num_shiftY;
	rep->list_shiftXY_scaled.resize(rep->num_shiftXY);

	vector<Point2i>().swap(gcLabels); // free memory
	for (int i_s = 0; i_s < rep->num_shiftX; i_s++)
	{
		for (int j_s = 0; j_s < rep->num_shiftY; j_s++)
		{
			Point2i* temp = new Point2i(rep->list_shiftX_scaled[i_s], rep->list_shiftY_scaled[j_s]);
			//Point2i* temp = new Point2i( i_s * rep->dist_shiftX_scaled, j_s * rep->dist_shiftY_scaled);
			rep->list_shiftXY_scaled[i_s * rep->num_shiftY + j_s] = *temp;
			gcLabels.push_back(rep->list_shiftXY_scaled[i_s * rep->num_shiftY + j_s]);
		}
	}
	// update gcGuide
	// find which label in list_shiftXY_scaled need to be used as hard constraint
	for (int i_rep = 0; i_rep < bbitem.size(); i_rep++)
	{
		for (int j_rep = 0; j_rep < bbitem[i_rep].size(); j_rep++)
		{
			if (bbitem[i_rep][j_rep]->flag_changed)
			{
				auto loc = std::find(rep->list_shiftXY_scaled.begin(), rep->list_shiftXY_scaled.end(), Point2i(bbitem[i_rep][j_rep]->x_shift, bbitem[i_rep][j_rep]->y_shift));
				int idx_shift = loc - rep->list_shiftXY_scaled.begin();
				Range r_y = Range(rep->repY_scaled[bbitem[i_rep][j_rep]->bb_type][bbitem[i_rep][j_rep]->bb_idx] + bbitem[i_rep][j_rep]->y_shift, rep->repY_scaled[bbitem[i_rep][j_rep]->bb_type][bbitem[i_rep][j_rep]->bb_idx] + rep->repH_scaled[bbitem[i_rep][j_rep]->bb_type][bbitem[i_rep][j_rep]->bb_idx] + bbitem[i_rep][j_rep]->y_shift);
				Range r_x = Range(rep->repX_scaled[bbitem[i_rep][j_rep]->bb_type][bbitem[i_rep][j_rep]->bb_idx] + bbitem[i_rep][j_rep]->x_shift, rep->repX_scaled[bbitem[i_rep][j_rep]->bb_type][bbitem[i_rep][j_rep]->bb_idx] + rep->repW_scaled[bbitem[i_rep][j_rep]->bb_type][bbitem[i_rep][j_rep]->bb_idx] + bbitem[i_rep][j_rep]->x_shift);	
				r_y.start = min(rep->rowsSyn_scaled - 1, max(0, r_y.start));
				r_x.start = min(rep->colsSyn_scaled - 1, max(0, r_x.start));
				r_y.end = min(rep->rowsSyn_scaled - 1, max(0, r_y.end));
				r_x.end = min(rep->colsSyn_scaled - 1, max(0, r_x.end));
				gcGuide(r_y, r_x) = idx_shift;
				ini_label = idx_shift;
			}
		}
	}
	for (int i_rep = 0; i_rep < bbitem.size(); i_rep++)
	{
		for (int j_rep = 0; j_rep < bbitem[i_rep].size(); j_rep++)
		{
			bbitem[i_rep][j_rep]->flag_changed = false;
		}
	}
	if (flag_MS)
	{
		synSampleMSL();
	} 
	else
	{
		synSample();
	}
}

void ImageViewer::changetypeBB(){
	myscribbleArea->setPenColor(qRgb(colorList[sel_bb_type][0], colorList[sel_bb_type][1], colorList[sel_bb_type][2]));

	myscribbleArea->setPenWidth(max(25, max(rep->repW_fullres[sel_bb_type][0] - 10, rep->repH_fullres[sel_bb_type][0] - 10)));
	//myscribbleArea->setPenColor(qRgb(122, 122, 122));
}

void ImageViewer::showInput(){
	//qDebug()<<"showInput started";
	//renderGuide();

	flag_sideselection = true;

	newscene = new QGraphicsScene();
	QImage* qimgtemp = new QImage(filename_imgInput);
	QGraphicsPixmapItem* imgDispInput = new QGraphicsPixmapItem(QPixmap::fromImage(*qimgtemp));
	newscene->addItem(imgDispInput);
	newscene->setSceneRect(0, 0, rep->qimgInput_fullres->width(), rep->qimgInput_fullres->height());
	newview = new QGraphicsView(newscene);
	newviewer = new sideviewer;
	newviewer->setCentralWidget(newview);
	newviewer->show();
	newviewer->resize(rep->qimgInput_fullres->width() + 10, rep->qimgInput_fullres->height() + 50);
	newviewer->setWindowTitle(tr("ImageInput"));

	// show all BB from the detection
	vector<vector<BBItem*>>().swap(bbitemInput);
	bbitemInput.resize(rep->numRep);
	for (int i_rep = 0; i_rep < rep->numRep; i_rep++)
	{
		for (int j_rep = 0; j_rep < rep->repX_fullres[i_rep].size(); j_rep++)
		{
			int X = rep->repX_fullres[i_rep][j_rep];
			int Y = rep->repY_fullres[i_rep][j_rep];
			int W = rep->repW_fullres[i_rep][j_rep];
			int H = rep->repH_fullres[i_rep][j_rep];
			QPolygon polygon;
			polygon << QPoint(X, Y)
				<< QPoint(X, Y + H)
				<< QPoint(X + W, Y + H)
				<< QPoint(X + W, Y);

			BBItem *item1 = new BBItem(0, 0);
			item1->setPolygon(polygon);
			item1->x_start = 0;
			item1->y_start = 0;
			item1->x_shift = 0;
			item1->y_shift = 0;
			item1->x_shift_ori = 0;
			item1->y_shift_ori = 0;
			item1->bb_type = i_rep;
			item1->bb_idx = j_rep;

			bbitemInput[i_rep].push_back(item1);
			int cur_idx = bbitemInput[i_rep].size() - 1;
			bbitemInput[i_rep][cur_idx]->setPen(pen);
			bbitemInput[i_rep][cur_idx]->setBrush( QColor(colorList[i_rep][0], colorList[i_rep][1], colorList[i_rep][2]) );
			bbitemInput[i_rep][cur_idx]->setZValue(qrand()%256);
			bbitemInput[i_rep][cur_idx]->setOpacity(0.75);
			bbitemInput[i_rep][cur_idx]->setFlag(QGraphicsItem::ItemIsMovable, false);
			newscene->addItem(bbitemInput[i_rep][cur_idx]);
		}
	}

	//qDebug()<<"showInput finished";
}

void ImageViewer::scribble(){

	qDebug()<<"to scribble";
	flag_syn = false;
	flag_scribble = true;

	vector<vector<BBItem*>>().swap(bbitem);
	vector<int>().swap(rep->rec_shift_list);
	vector<vector<pair<int, int>>>().swap(rep->rec_bb_list);
	scalerSynX = scalerPaintX; // scaling factor for synthesized image, X dimension
	scalerSynY = scalerPaintY; // scaling factor for synthesized image, Y dimension

	// and initialize an empty canvas
	rep->rowsSyn_scaled = round(rep->rowsInput_scaled * scalerPaintY);
	rep->colsSyn_scaled = round(rep->colsInput_scaled * scalerPaintX);
	rep->rowsSyn_fullres = rep->rowsSyn_scaled/scalerRes;
	rep->colsSyn_fullres = rep->colsSyn_scaled/scalerRes;
	global_rowsSyn_fullres = rep->rowsSyn_fullres;
	global_colsSyn_fullres = rep->colsSyn_fullres;
	global_rowsSyn_scaled = rep->rowsSyn_scaled;
	global_colsSyn_scaled = rep->colsSyn_scaled;
	inputcols = rep->colsInput_scaled;
	inputrows = rep->rowsInput_scaled;
	rep->numPixelSyn_scaled = rep->rowsSyn_scaled * rep->colsSyn_scaled;

	rep->imgSyn_fullres = Mat3d::zeros(rep->rowsSyn_fullres, rep->colsSyn_fullres) + Scalar(255,255,255);
	*rep->qimgSyn_fullres = Mat2QImage(rep->imgSyn_fullres);
	imgDisp = new QGraphicsPixmapItem(QPixmap::fromImage(*rep->qimgSyn_fullres));

	rep->imgSynGray_scaled = Mat1b::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled); //or, rep->imgSynGray_scaled.create(rows, cols);
	rep->gcolabelSyn_scaled = Mat1b::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled);

	myscribbleArea = new ScribbleArea;
	myscribbleArea->image = Mat2QImage(rep->imgSyn_fullres);
	// put myscribbleArea in the central 
	setCentralWidget(myscribbleArea);
	setWindowTitle(tr("Scribble"));
	
	resize(rep->qimgSyn_fullres->width(), rep->qimgSyn_fullres->height());

	//and open a sideviewer for selecting bb type
	showInput();
	flag_scribblemap = true;
}

void ImageViewer::synScribble(){
	if (!flag_scribblemap)
		return;
	flag_scribblemap = false;
	qDebug()<<"guideStitch ... ";
	synmode = 3;
	flag_interactive = false;
	flag_scribble = true;
	QImage visibleImage = myscribbleArea->image;
	Mat3b visibleMat = Mat3b::zeros(rep->rowsSyn_fullres, rep->colsSyn_fullres);
	visibleMat = qimage2mat(visibleImage);
	rep->imgSyn_fullres = qimage2mat(visibleImage);

	// compute label index map 
	Mat1b maplabel = Mat1b::zeros(rep->rowsSyn_fullres, rep->colsSyn_fullres);
	Mat channel[3];
	Mat mask;
	split(rep->imgSyn_fullres, channel);
	for (int i_rep = 0; i_rep < rep->numRep; i_rep++)
	{
		mask = (channel[0] == colorList[i_rep][2] & channel[1] == colorList[i_rep][1] & channel[2] == colorList[i_rep][0]);
		maplabel += mask * (i_rep + 1)/255;
	}


	gcGuideScribble = Mat1b::zeros(rep->rowsSyn_fullres, rep->colsSyn_fullres);
	gcGuideScribble = gcGuideScribble - 1;
	gcGuideMask = Mat1b::zeros(rep->rowsSyn_scaled, rep->colsSyn_scaled);

	cv::resize(maplabel, rep->imgGuide_scaled, Size(rep->colsSyn_scaled,rep->rowsSyn_scaled), 0, 0, INTER_NEAREST);
	cv::resize(maplabel, gcGuideScribble, Size(rep->colsSyn_scaled,rep->rowsSyn_scaled), 0, 0, INTER_NEAREST);

	//for (int i = 0; i < 10; i++)
	//{
	//	for (int j = 0; j < 10; j++)
	//	{
	//		qDebug()<<gcLabel(i, j);
	//	}
	//}
	//imshow("gcLabel", gcLabel * 50);
	//imshow("gcGuideScribble", gcGuideScribble * 50);

	// generate candidate shifts
	int startX = 0;
	int startY = 0;
	rep->num_shiftX = startX;
	rep->num_shiftY = startY;
	vector<int>().swap(rep->list_shiftX_scaled);
	vector<int>().swap(rep->list_shiftY_scaled);
	vector<int>().swap(rep->list_shiftX_fullres);
	vector<int>().swap(rep->list_shiftY_fullres);
	vector<Point2i>().swap(rep->list_shiftXY_scaled);
	vector<Point2i>().swap(rep->list_shiftXY_fullres);


	// well, we choose the starting location of expanding occurrence statistics from [0, 0] 
	rep->list_shiftX_scaled.push_back(0);
	rep->list_shiftY_scaled.push_back(0);
	rep->num_shiftX++;
	rep->num_shiftY++;

	// then expand the remaining list using statistical sampling
	// first, compute the minimum shift that cover the whole image
	int range_shift_x = rep->colsSyn_scaled - rep->colsInput_scaled;
	int range_shift_y = rep->rowsSyn_scaled - rep->rowsInput_scaled;

	// expand in x direction ... 
	// do insertion at the front, for efficiency should use deque, but doesn't matter for now
	while(rep->list_shiftX_scaled[0] > x_shift_lowbd){
		// sample a shift
		//int gen_X = rand() % rep->num_bbstatisticsX;
		//rep->expansion_bbstatisticsX = rep->list_bbstatisticsX[gen_X];
		//rep->expansion_bbstatisticsX_scaled = round(rep->expansion_bbstatisticsX * scalerRes);
		rep->expansion_bbstatisticsX_scaled = 8;
		rep->list_shiftX_scaled.insert(rep->list_shiftX_scaled.begin(),rep->list_shiftX_scaled[0] - rep->expansion_bbstatisticsX_scaled);
		rep->num_shiftX += 1;
	}
	// do insertion at the end

	while(rep->list_shiftX_scaled[rep->list_shiftX_scaled.size() - 1] < range_shift_x + x_shift_upbd){
		//int gen_X = rand() % rep->num_bbstatisticsX;
		//rep->expansion_bbstatisticsX = rep->list_bbstatisticsX[gen_X];
		//rep->expansion_bbstatisticsX_scaled = round(rep->expansion_bbstatisticsX * scalerRes);
		rep->expansion_bbstatisticsX_scaled = 8;
		rep->list_shiftX_scaled.push_back(rep->list_shiftX_scaled[rep->list_shiftX_scaled.size() - 1] + rep->expansion_bbstatisticsX_scaled);
		rep->num_shiftX += 1;
	}

	while(rep->list_shiftY_scaled[0] > y_shift_lowbd){
		// sample a shift
		//int gen_Y = rand() % rep->num_bbstatisticsY;
		//rep->expansion_bbstatisticsY = rep->list_bbstatisticsY[gen_Y];
		//rep->expansion_bbstatisticsY_scaled = round(rep->expansion_bbstatisticsY * scalerRes);
		rep->expansion_bbstatisticsY_scaled = 8;
		rep->list_shiftY_scaled.insert(rep->list_shiftY_scaled.begin(),rep->list_shiftY_scaled[0] - rep->expansion_bbstatisticsY_scaled);
		rep->num_shiftY += 1;
	}
	// do insertion at the end

	while(rep->list_shiftY_scaled[rep->list_shiftY_scaled.size() - 1] < range_shift_y + y_shift_upbd){
		//int gen_Y = rand() % rep->num_bbstatisticsY;
		//rep->expansion_bbstatisticsY = rep->list_bbstatisticsY[gen_Y];
		//rep->expansion_bbstatisticsY_scaled = round(rep->expansion_bbstatisticsY * scalerRes);
		rep->expansion_bbstatisticsY_scaled = 8;
		rep->list_shiftY_scaled.push_back(rep->list_shiftY_scaled[rep->list_shiftY_scaled.size() - 1] + rep->expansion_bbstatisticsY_scaled);
		rep->num_shiftY += 1;
	}

	// compute the candidate shifts
	rep->num_shiftXY = rep->num_shiftX * rep->num_shiftY;
	rep->list_shiftXY_scaled.resize(rep->num_shiftXY);

	// set the gco initial label to the shift of [startX, startY]
	auto loc = std::find(rep->list_shiftXY_scaled.begin(), rep->list_shiftXY_scaled.end(), Point2i(startX, startY));
	int idx_shift = loc - rep->list_shiftXY_scaled.begin();
	ini_label = idx_shift;

	if (flag_MS)
	{
		synSampleMSL();
	} 
	else
	{
		synSample();
	}


}

void ImageViewer::save(){
	QString imagePath = QFileDialog::getSaveFileName(
		this,
		tr("Save File"),
		"",
		tr("PNG (*.png)" )
		);
	if (renderRepLabelflag == 1)
	{
		rep->qimgSynlabelColor_fullres->save(imagePath);
	} 
	else
	{
		rep->qimgSyn_fullres->save(imagePath);
	}
	
}

void ImageViewer::print(){


}