#include "util.h"

//追加作成

using namespace ev3api;

class PrmReq{
public:
	PrmReq();
//	void init();
	void setPRM_a(int prm);
	int getPRM_a(void);

 private:
 	int a = 0;
 	int b = 0;
};
