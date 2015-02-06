#include "SenderObject.h"
#include <QDebug>

void SenderObject::sendsignal(){
	emit itemmoved();
}

void SenderObject::sendsignaladditem(){
	emit itemadded();
}

void SenderObject::sendsinglechangebbtype(){
	emit bbtypechanged();
}