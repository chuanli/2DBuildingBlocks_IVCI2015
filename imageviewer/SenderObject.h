#ifndef RECTOBJECT_H
#define RECTOBJECT_H
#include <QObject>
// a minimum signal sender inherited from QObject
class SenderObject : public QObject
{
	Q_OBJECT

public:
	SenderObject(){}

	void sendsignal();
	void sendsignaladditem();
	void sendsinglechangebbtype();
signals:
	void itemmoved(); // do not need to implement anything here, just a signal
	void itemadded();
	void bbtypechanged();
};


#endif