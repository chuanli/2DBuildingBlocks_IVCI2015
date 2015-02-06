#ifndef SIDEVIEWER_H
#define SIDEVIEWER_H
#include <QMainWindow>
#include <QAction>
#include <QMenu>
#include <QKeyEvent>

#include "Para.h"


class sideviewer: public QMainWindow
{
public:
	sideviewer();
	//void closewithupdate();

	QMenu *fileMenu;

protected:
	void keyPressEvent(QKeyEvent *event);
private:

	QAction *exitAct;
};


#endif