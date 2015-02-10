#include "Rep.h"
#include <QDebug>

inline double round( double d )
{
	return floor( d + 0.5 );
}

BBItem::BBItem(int a, int b):idx_s(a), idx_item(b){
	flag_changed = false;
	setFlag(ItemIsMovable);
	setFlag(ItemIsSelectable);
	setFlag(ItemSendsGeometryChanges);
	setCacheMode(DeviceCoordinateCache);
	setZValue(-1);
}

void BBItem::mousePressEvent(QGraphicsSceneMouseEvent *event){
	flag_changed = true;
	QGraphicsPolygonItem::mousePressEvent(event);
	if (flag_sideselection)
	{
		if (flag_scribble)
		{
			//pick up this color
			sel_bb_type = bb_type;
			globalsender->sendsinglechangebbtype();
		} 
	}
	else{
		qDebug()<<"mouse pressed"<<", x: "<<this->x()<<", y: "<<this->y();
		//sel_bb_type = bb_type;
		//globalsender->sendsinglechangebbtype();
	}
}

void BBItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){
	if (flag_syn)
	{
		updateItem(event);
	}
}

void BBItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event){
	update();
	QGraphicsPolygonItem::mouseMoveEvent(event);
}

void BBItem::updateItem(QGraphicsSceneMouseEvent *event){
	// update a single item
	int newx = round((double)this->x() * scalerRes)/scalerRes; // it is necessary to round up for gco in the low resolution
	int newy = round((double)this->y() * scalerRes)/scalerRes;

	// need to make sure do not cross input image boundary
	qDebug()<<"before boundary check, newx "<<newx<<", w_fullres:"<<w_fullres<<", x_fullres: "<<x_fullres<<", x_shift_ori: "<<x_shift_ori/scalerRes;
	qDebug()<<"before boundary check, newy "<<newy<<", h_fullres:"<<h_fullres<<", y_fullres: "<<y_fullres<<", y_shift_ori: "<<y_shift_ori/scalerRes;

	if ((newx + x_fullres + x_shift_ori/scalerRes) < 0)
	{
		newx = round((double)(-x_fullres - x_shift_ori/scalerRes) * scalerRes + 1)/scalerRes;
	} 
	else if (newx + w_fullres + x_fullres + x_shift_ori/scalerRes >= global_colsSyn_fullres)
	{
		newx = round((double)(global_colsSyn_fullres - x_fullres - w_fullres - x_shift_ori/scalerRes) * scalerRes - 1)/scalerRes;
	}

	if ((newy + y_fullres + y_shift_ori/scalerRes) < 0)
	{
		newy = round((double)(-y_fullres - y_shift_ori/scalerRes) * scalerRes + 1)/scalerRes;
	} 
	else if (newy + h_fullres + y_fullres + y_shift_ori/scalerRes >= global_rowsSyn_fullres)
	{
		newy = round((double)(global_rowsSyn_fullres - y_fullres - h_fullres - y_shift_ori/scalerRes) * scalerRes - 1)/scalerRes;
	}

	qDebug()<<"after boundary check, newx "<<newx<<", w_fullres:"<<w_fullres<<", x_fullres: "<<x_fullres<<", x_shift_ori: "<<x_shift_ori/scalerRes;
	qDebug()<<"after boundary check, newy "<<newy<<", h_fullres:"<<h_fullres<<", y_fullres: "<<y_fullres<<", y_shift_ori: "<<y_shift_ori/scalerRes;

	//this->setPos(newx, newy);
	this->setPos(newx, this->y_start);
	update();
	QGraphicsPolygonItem::mouseReleaseEvent(event);
	globalsender->sendsignal();	
}


//
//void BBItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
//{
//	
//	//update();
//	flag_changed = true;
//	QGraphicsPolygonItem::mousePressEvent(event);
//	//qDebug()<<"mouse pressed"<<", x: "<<this->x()<<", y: "<<this->y();
//	qDebug()<<flag_sideselection;
//	if (flag_sideselection)
//	{
//		if (flag_scribble)
//		{
//			//qDebug()<<"to pick up this color";
//			sel_bb_type = bb_type;
//			globalsender->sendsinglechangebbtype();
//		} 
//		else
//		{
//			// add this item to the main scene
//			// need to record i_rep and j_rep and send a signal to mainwindow for update
//			sel_bb_type = bb_type;
//			sel_bb_idx = bb_idx;
//			//qDebug()<<bb_type<<bb_idx;
//			globalsender->sendsignaladditem();
//		}
//
//	}
//}
//
//
//void BBItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
//{
//	if (flag_syn_Y)
//	{
//		updateY(event);
//	} 
//	else
//	{
//		updateX(event);
//	}
//}
//
//void BBItem::updateX(QGraphicsSceneMouseEvent *event){
//
//	qDebug()<<"mouse released"<<", x: "<<this->x()<<", y: "<<this->y();
//	// need to rectify the position of this item
//	// first, the y position should not be changed
//	if (flag_sideselection)
//	{
//
//	} 
//	else
//	{
//		if (flag_multiselection)
//		{
//			// update all selected items
//			QList<QGraphicsItem *> itemSelected = this->scene()->selectedItems();
//			double min_x = 1000000;
//			double max_x = -1;
//			for (int i_item = 0; i_item < itemSelected.size(); i_item++)
//			{
//				QRectF Recttemp = itemSelected[i_item]->boundingRect();
//				min_x = qMin(min_x, round((Recttemp.x() + itemSelected[i_item]->x()) * scalerRes - 1)/scalerRes);
//				max_x = qMax(max_x, round((Recttemp.x() + Recttemp.width() + itemSelected[i_item]->x()) * scalerRes + 1)/scalerRes);
//			}
//			int shift_correction = 0;
//			if (min_x < 0 )
//			{
//				shift_correction = -min_x;
//			} 
//			else if (max_x >= global_colsSyn_fullres)
//			{
//				shift_correction = global_colsSyn_fullres - max_x - 1;
//			}
//
//			qDebug()<<min_x<<max_x<<shift_correction;
//
//			for (int i_item = 0; i_item < itemSelected.size(); i_item++)
//			{
//				int newx = round(((double)itemSelected[i_item]->x() + shift_correction) * scalerRes)/scalerRes;
//				itemSelected[i_item]->setPos(newx, 0);
//			}
//			update();
//			QGraphicsPolygonItem::mouseReleaseEvent(event);
//		}
//		else{
//			// update a single item
//			int newx = round((double)this->x() * scalerRes)/scalerRes;
//			// need to make sure do not cross input image boundary
//			qDebug()<<"before boundary check: "<<newx<<", w_fullres:"<<w_fullres<<", x_fullres: "<<x_fullres<<", x_shift_ori: "<<x_shift_ori/scalerRes;
//
//			if ((newx + x_fullres + x_shift_ori/scalerRes) < 0)
//			{
//				newx = round((double)(-x_fullres - x_shift_ori/scalerRes) * scalerRes + 1)/scalerRes;
//			} 
//			else if (newx + w_fullres + x_fullres + x_shift_ori/scalerRes >= global_colsSyn_fullres)
//			{
//				newx = round((double)(global_colsSyn_fullres - x_fullres - w_fullres - x_shift_ori/scalerRes) * scalerRes - 1)/scalerRes;
//			}
//			qDebug()<<"after boundary check: "<<newx<<", w_fullres:"<<w_fullres<<", x_fullres: "<<x_fullres<<", x_shift_ori: "<<x_shift_ori/scalerRes;
//
//			this->setPos(newx, this->y_start);
//			update();
//			QGraphicsPolygonItem::mouseReleaseEvent(event);
//			globalsender->sendsignal();
//		}
//	}
//	//qDebug()<<"mouse released"<<", x: "<<this->x()<<", y: "<<this->y();
//	// // send signal to QMainwindow
//}
//
//void BBItem::updateY(QGraphicsSceneMouseEvent *event){
//	qDebug()<<"mouse released"<<", x: "<<this->x()<<", y: "<<this->y();
//	// need to rectify the position of this item
//	// first, the y position should not be changed
//	if (flag_sideselection)
//	{
//
//	} 
//	else
//	{
//		if (flag_multiselection)
//		{
//			// update all selected items
//			QList<QGraphicsItem *> itemSelected = this->scene()->selectedItems();
//			double min_y = 1000000;
//			double max_y = -1;
//			for (int i_item = 0; i_item < itemSelected.size(); i_item++)
//			{
//				QRectF Recttemp = itemSelected[i_item]->boundingRect();
//				min_y = qMin(min_y, round((Recttemp.y() + itemSelected[i_item]->y()) * scalerRes - 1)/scalerRes);
//				max_y = qMax(max_y, round((Recttemp.y() + Recttemp.width() + itemSelected[i_item]->y()) * scalerRes + 1)/scalerRes);
//			}
//
//			int shift_correction = 0;
//			if (min_y < 0 )
//			{
//				shift_correction = -min_y;
//			} 
//			else if (max_y >= global_rowsSyn_fullres)
//			{
//				shift_correction = global_rowsSyn_fullres - max_y - 1;
//			}
//
//			qDebug()<<min_y<<max_y<<shift_correction;
//
//			for (int i_item = 0; i_item < itemSelected.size(); i_item++)
//			{
//				int newy = round(((double)itemSelected[i_item]->y() + shift_correction) * scalerRes)/scalerRes;
//				itemSelected[i_item]->setPos(0, newy);
//			}
//
//			update();
//			QGraphicsPolygonItem::mouseReleaseEvent(event);
//		}
//		else{
//			// update a single item
//			int newy = round((double)this->y() * scalerRes)/scalerRes;
//			// need to make sure do not cross input image boundary
//			qDebug()<<"before boundary check: "<<newy<<", h_fullres:"<<h_fullres<<", y_fullres: "<<y_fullres<<", y_shift_ori: "<<y_shift_ori/scalerRes;
//
//			if ((newy + y_fullres + y_shift_ori/scalerRes) < 0)
//			{
//				newy = round((double)(-y_fullres - y_shift_ori/scalerRes) * scalerRes + 1)/scalerRes;
//			} 
//			else if (newy + h_fullres + y_fullres + y_shift_ori/scalerRes >= global_rowsSyn_fullres)
//			{
//				newy = round((double)(global_rowsSyn_fullres - y_fullres - h_fullres - y_shift_ori/scalerRes) * scalerRes - 1)/scalerRes;
//			}
//			qDebug()<<"after boundary check: "<<newy<<", h_fullres:"<<h_fullres<<", y_fullres: "<<y_fullres<<", y_shift_ori: "<<y_shift_ori/scalerRes;
//
//			this->setPos(this->x_start, newy);
//			update();
//			QGraphicsPolygonItem::mouseReleaseEvent(event);
//			globalsender->sendsignal();
//		}
//	}
//	//qDebug()<<"mouse released"<<", x: "<<this->x()<<", y: "<<this->y();
//	// // send signal to QMainwindow
//}
//
//void BBItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
//{
//	//int newx = round((double)this->x() * scalerRes)/scalerRes;
//	//this->setPos(newx, this->y_start);
//	update();
//	QGraphicsPolygonItem::mouseMoveEvent(event);
//} 

Rep::Rep(void)
{
	// Input image
	qimgInput_fullres = NULL;
	qimgInput_scaled = NULL;
	qimgInputGray_fullres = NULL;
	qimgInputGray_scaled = NULL;
	qlabelInput_fullres = NULL;
	qlabelInput_scaled = NULL;
	qlabelInputGray_fullres = NULL;
	qlabelInputGray_scaled = NULL;

	// Input label
	qimgInputlabel_fullres = NULL;
	qimgInputlabel_scaled = NULL;

	// Input internal label
	qimgInputlabelinterX_fullres = NULL;
	qimgInputlabelinterX_scaled = NULL;

	qimgInputlabelinterY_fullres = NULL;
	qimgInputlabelinterY_scaled = NULL;

	// Synthesized image
	qimgSyn_fullres = NULL;
	qimgSyn_scaled = NULL;
	qimgSynGray_fullres = NULL;
	qimgSynGray_scaled = NULL;
	qlabelSyn_fullres = NULL;
	qlabelSyn_scaled = NULL;
	qlabelSynGray_fullres = NULL;
	qlabelSynGray_scaled = NULL;

	qimgSynlabelColor_fullres = NULL;
	qlabelSynlabelColor_fullres = NULL;


	// user guidance
	qimgUserGuide_fullres = NULL;
	qimgUserGuide_scaled = NULL;
	qimgUserGuideGray_fullres = NULL;
	qimgUserGuideGray_scaled = NULL;
	//data related to guidance map -- not sure what is this thing ... 

	// dimension
	rowsInput_scaled = 0;
	colsInput_scaled = 0;
	rowsSyn_scaled = 0;
	colsSyn_scaled = 0;
	rowsPaint_scaled = 0;
	colsPaint_scaled = 0;
	numPixelInput_scaled = 0;
	numPixelSyn_scaled = 0;

	rowsInput_fullres = 0;
	colsInput_fullres = 0;
	rowsSyn_fullres = 0;
	colsSyn_fullres = 0;
	rowsPaint_fullres = 0;
	colsPaint_fullres = 0;
	numPixelInput_fullres = 0;
	numPixelSyn_fullres = 0;

	// shifts
	num_shiftX = 0;
	dist_shiftX_scaled = 0;
	dist_shiftX_fullres = 0;
	num_shiftY = 0;
	dist_shiftY_scaled = 0;
	dist_shiftY_fullres = 0;
	num_shiftXY = 0;

	// shift for MSL
	num_shiftMSL = 0;

	// Repetitive Building Blocks data ... 
	numRep = 0;
	numBB = 0;

	// co-occurrence statistics
	numCooC = 0;

	// co-occurrent statistics of BB
	num_bbstatisticsX = 0;
	num_bbstatisticsY = 0;
	expansion_bbstatisticsX = 0;
	expansion_bbstatisticsY = 0;
	expansion_bbstatisticsX_scaled = 0;
	expansion_bbstatisticsY_scaled = 0;

	// votes
	imgVote_scaled = NULL;

	// a constellation distance matrix for pairwise pixels
}

Rep::~Rep(void)
{
}
