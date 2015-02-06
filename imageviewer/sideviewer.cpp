#include "sideviewer.h"

sideviewer::sideviewer(){
	//exitAct = new QAction(tr("E&xit"), this);
	//exitAct->setShortcut(tr("Ctrl+Q"));
	//connect(exitAct, SIGNAL(triggered()), this, SLOT(closewithupdate()));

	//fileMenu = new QMenu(tr("&File"), this);
	//fileMenu->addAction(exitAct);
}


//void sideviewer::closewithupdate(){
//
//	flag_sideselection = false;
//	this->close();
//}


void sideviewer::keyPressEvent(QKeyEvent* event)
{
	switch (event->key()) {
	case Qt::Key_Up:
		break;
	case Qt::Key_Down:
		break;
	case Qt::Key_Left:
		break;
	case Qt::Key_Right:
		break;
	case Qt::Key_Plus:
		break;
	case Qt::Key_Minus:
		break;
	case Qt::Key_Escape:
		{
			qDebug()<<"Close input image and go back to the editor.";
			flag_sideselection = false;
			flag_scribble = false;
			close();
		}
	case Qt::Key_Enter:
		break;
	default:
		QMainWindow::keyPressEvent(event);
	}
}