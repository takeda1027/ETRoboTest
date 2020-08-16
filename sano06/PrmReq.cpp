#include "PrmReq.h"
//using namespace ev3api;

//���q�ǉ��N���X

//PrmReq::PrmReq(){}

//void PrmReq::init() {
//  init_f("PrmReq");
//}

void PrmReq::setPRM_a(int prm){
	
	a = prm;
}

int  PrmReq::getPRM_a(void)
{
	return a;
}

