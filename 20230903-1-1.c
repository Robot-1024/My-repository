/************端口定义************/
#define gd1 getadc(9)        //左地灰 
#define gd2 getadc(10)       //左前地灰 
#define gd3 getadc(11)		 //右前地灰 
#define gd4 getadc(12)		 //右地灰 
#define gd_fr1 getadc(13)	 //黄线地灰1 
#define gd_fr2 getadc(14)	 //黄线地灰2 
#define irr getadc(1)		 //右前测距 
#define irf getadc(2)		 //前测距 
#define senor_qr getadc(3)   //二维码模块
#define irr_cz getadc(4)     //红外测障
#define Open_MV getadc(5)    //OpenMV摄像头 
#define cps compass()		 //数字指南针[后换陀螺仪] 
#define icl getport(5)       //滚珠开关 

#define sev_gd 1             //控制右前方地灰的舵机
#define sev_1 2              //机械臂舵机
#define sev_2 3              //机械爪舵机
#define sev_blocks 6

/************子函数定义************/
void go_line(float x);               //沿右侧黄线行进，X为行进时间
void go_fr_t(float time);            //沿墙走一段时间
void go_fr_s(float x);               //沿右侧墙低速走一段时间或到路口
void goto_intersection(void);        //前进到路口（gd2 3检测白线）
void goto_barrier(int x);            //前进到前方护栏
void intersection_handling(int x);   //路口处理
void intersection_correct(void);     //路口矫正
void sev_rst(void);                  //舵机初始化
void sev_catch_up(int x);            //抓取物块
void sev_put_down(int x);            //放物块
void red_led_light(void);            //红绿灯路口检测
void Auto_brake(int x, int y);       //自动防撞
void turn_P(double targetAngle);     //旋转
void ground_test(void);              //测试阈值
void SpaceBase(void);                //投放物资
void LabCenter(void);
void run(int a, int b, int c, int d);           //4轮独立控制
void run_t(int a, int b, int c, int d, double t); //4轮独立控制计时
void run_f(int x, int y);                       //直排轮控制模式
void run_ft(int x, int y, double t);            //直排轮控制模式计时
void stop(void);                                //停止运动
void go_line_P(float x);                        //P控制走黄线
void EE_wr(int a, int b);                       //写入数据
int  EE_re(int a);                              //读取数据
void ground_test_input(void);                   //输入数据
void ground(int mode);                          //初始化地灰
void goto_intersection_new(void);               //新 走到路口
void go_fr_tp(float time);                      //沿墙走一段时间[带平移]
void intersection_right(int degree);            //路口右转
void intersection_left(int degree);             //路口左转
void sev_gd_down(void);                         //放下地灰
void intersection_turn(int t1, int t2);         //路口掉头
void test(int n);                               //测试
void catch_blocks(int m);                       //抓取方块

/*红外测距参考值*/
int irr_4 = 500, irr_6 = 450, irr_9 = 385, irr_12 = 300; //右前 4cm 6cm 9cm 12cm
int irr_null = 200;
int irf_8 = 490, irf_brake = 460;               //前 8cm

/*地灰参考值*/
int gd_m1, gd_m2, gd_m3, gd_m4; //地面四地灰
int gdfr_m1, gdfr_m2;         //黄线地灰
int gd_m3_b;                  //右前地灰悬空

int i_brake, i_QR, i_SpaceBase = 0, i_LabCenter = 0;
int radian = 1;
float t_brake;

//舵机参数
int s_gd_up = 200, s_gd_down = 1850; //地灰舵机抬起放下参考值

int s_up = 2550;                //机械臂舵机抬升参考值
int s_down = 950;               //机械臂舵机放下参考值
int s_catch = 1050;             //手抓舵机抓取参考值
int s_open = 1500;              //手抓舵机打开参考值
int s_push = 2700;
int s_back = 1800;

/**********************主函数**********************/

int main(void) {
	//舵机初始化，再按运行键启动
	wait(0.5);
	sev_rst();
	ground(1);

	/******蓝色路线部分******/

	//左转过C路口
	go_fr_tp(0.5);
	goto_intersection_new();
	intersection_correct();
	intersection_left(-77);

	//右转过D路口
	go_fr_tp(0.3);
	goto_intersection_new();
	intersection_correct();
	intersection_right(79);

	//右转过F路口
	go_fr_tp(0.3);
	goto_intersection_new();
	intersection_correct();
	intersection_right(86);

	//直行过H路口
	//抓取1号物块
	//code for big blocks
	go_fr_tp(0.3);
	red_led_light();
	goto_intersection_new();
	intersection_correct();
	intersection_handling(1);



	//右转过G路口
	//抓取2号物块
	goto_intersection_new();
	intersection_correct();
	intersection_right(98);

	//直行过B路口
	go_fr_tp(0.3);
	goto_intersection_new();
	intersection_correct();
	intersection_handling(11);

	go_line(gd_m2);
	while (irf < irf_8) run_f(300, 300);
	run_ft(-300, -300, 0.1);
	run_ft(-350, 350, 0.37);
	while (gd3 < gd_m3_b) go_fr_s(0.01);
	run_ft(-500, -500, 0.1);
	run_ft(-350, 350, 0.37);
	run_ft(500, 500, 0.1);
	servo(sev_gd, s_gd_down);
	go_line(gd_m2);


	/******绿色路线部分******/

	
	//左转过B路口
	intersection_left(-75);

	//直行过C路口
	//抓取3号方块
	//code for small blocks
	goto_intersection_new();
	intersection_correct();
	intersection_handling(1);

	//右转过D路口
	//抓取4号方块
	//code for small blocks
	goto_intersection_new();
	intersection_correct();
	intersection_right(83);

	/*
	
	//掉头过I路口
	//抓取5号方块
	//code for small blocks
	goto_intersection_new();
	intersection_correct();
	intersection_turn(-90, -90);

	//直行过D路口
	go_fr_tp(0.3);
	goto_intersection_new();
	intersection_correct();
	intersection_handling(1);

	  //code for three small blocks
	*/

	/******红色路线部分******/
	/*
	//直行过D路口
	intersection_handling(1);

	//右转过I路口
	go_fr_tp(0.3);
	goto_intersection_new();
	intersection_correct();
	intersection_right(90);


	//右转过H路口
	go_fr_tp(0.3);
	goto_intersection_new();
	intersection_correct();
	intersection_right(90);

	//掉头过C路口
	//抓取6号方块
	//code for small blocks
	goto_intersection_new();
	intersection_correct();
	intersection_turn(-90, -90);

	//右转过H路口
	//抓取7号方块
	//code for small blocks
	goto_intersection_new();
	intersection_correct();
	intersection_right(90);

	//右转过G路口
	go_fr_tp(0.3);
	goto_intersection_new();
	intersection_correct();
	intersection_right(90);

	//右转过B路口
	go_fr_tp(0.3);
	goto_intersection_new();
	intersection_correct();
	intersection_right(90);

	//直行过C路口
	go_fr_tp(0.3);
	goto_intersection_new();
	intersection_correct();
	intersection_handling(1);

	//左转过D路口
	go_fr_tp(0.3);
	goto_intersection_new();
	intersection_correct();
	intersection_left(-90);

	//code for three small blocks

	  go_fr_tp(0.3);*/

	/******黄色路线部分******/



	return 0;
}

/*****************行进路线子函数*****************/

void Space_3s(void) {
	servo(sev_gd, s_gd_down);
	go_line(0.8);
	servo(sev_gd, s_gd_up);
	run_f(240, 250);
	while (irf < irf_8);
	stop();
	sound(500, 100);
	run_ft(-350, 350, 0.35);
	sound(500, 100);
	while (irr < irr_6) {
		run(200, -250, 200, -250);
	};
	stop();
	sound(800, 100);
	for (int i = 1; i <= 3; i++) {
		go_fr_s(0.3);
		stop();
		sound(500, 100);
		sev_put_down(1);
	}
	go_fr_s(220);
	run_f(0, 0);
	sound(500, 100);
	//左转前进到黄边线
	run_f(-300, -300);
	wait(0.1);
	run_f(0, 0);
	sound(500, 200);
	run_f(-350, 350);
	wait(0.36);
	servo(sev_gd, s_gd_down);
	run(200, 400, 200, 400);
	wait(0.45);
	run_f(0, 0);
	sound(500, 100);
	while (gd_fr2 > gdfr_m2 && gd_fr1 > gdfr_m1) {
		run(300, 0, 300, 0);
	}
	run_f(0, 0);
	sound(500, 100);
	//沿黄边线到白线路口，并校正
	go_line(gd_m2);
	intersection_correct();//路口矫正
}


/****************************************
  函数名称:SpaceBase
  参数:/
  作用:投放物资1
  返回:/
  最后更新日期:2023/08/13
 *****************************************/
void SpaceBase(void) {
	go_line(0.8); //沿黄线走0.8秒
	servo(sev_gd, s_gd_up);
	wait(1); //抬起地灰
	//前进到投放区并左转
	run_f(240, 250);
	while (irf < irf_8);
	run_f(0, 0);
	sound(500, 100);
	run_f(-350, 350);
	wait(0.35);
	run_f(0, 0);
	sound(500, 100);
	//靠近平台
	while (irr < irr_6) {
		run(200, -250, 200, -250);
	}   //---通过irr6测距
	run_f(0, 0);
	sound(800, 100);

	//投放物块
	if (i_SpaceBase == 1) {
		go_fr_s(0.3);
		run_f(0, 0);
		sound(500, 100);
	}
	else if (i_SpaceBase == 2) {
		go_fr_s(0.4);
		run_f(0, 0);
		sound(500, 100);
	}
	else if (i_SpaceBase == 3) {
		go_fr_s(0.55);
		run_f(0, 0);
		sound(500, 100);
	}
	else if (i_SpaceBase == 4) {
		go_fr_s(0.7);
		run_f(0, 0);
		sound(500, 100);
	}
	else if (i_SpaceBase == 5) {
		go_fr_s(220);
		run_f(0, 0);
		sound(500, 100);
	}

	sev_put_down(1);
	i_SpaceBase++;
	go_fr_s(220);
	run_f(0, 0);
	sound(500, 100);

	//左转前进到黄边线
	run_f(-300, -300);
	wait(0.1);
	run_f(0, 0);
	sound(500, 200);
	run_f(-350, 350);
	wait(0.36);
	servo(sev_gd, s_gd_down);
	run(200, 400, 200, 400);
	wait(0.45);
	run_f(0, 0);
	sound(500, 100);

	while (gd_fr2 > gdfr_m2 && gd_fr1 > gdfr_m1) {
		run(300, 0, 300, 0);
	}

	run_f(0, 0);
	sound(500, 100);

	//沿黄边线到白线路口，并校正
	go_line(gd_m2);
	intersection_correct();//路口矫正
}

/****************************************
  函数名称:LabCenter
  参数:/
  作用:投放物资2
  返回:/
  最后更新日期:2023/08/13
 *****************************************/
void LabCenter(void) {
	go_line(0.8);
	servo(sev_gd, s_gd_up);
	wait(1);
	//前进到投放区域并左转
	run_f(240, 250);
	while (irf < irf_8);
	run_f(0, 0);
	sound(500, 100);
	run_f(-350, 350);
	wait(0.35);
	run_f(0, 0);
	sound(500, 100);

	//投放物块
	if (i_LabCenter == 1) {
		go_fr_s(0.5);
		run_f(0, 0);
		sound(500, 100);
	}

	sev_put_down(2);
	i_LabCenter++;
	go_fr_s(220);
	run_f(0, 0);
	sound(500, 100);

	//左转前进到黄边线
	run_f(-300, -300);
	wait(0.1);
	run_f(0, 0);
	sound(500, 200);
	run_f(-350, 350);
	wait(0.36);
	servo(sev_gd, s_gd_down);
	run(200, 400, 200, 400);
	wait(0.45);
	run_f(0, 0);
	sound(500, 100);
	while (gd_fr2 > gdfr_m2 && gd_fr1 > gdfr_m1) {
		run(300, 0, 300, 0);
	}
	run_f(0, 0);
	sound(500, 100);

	//沿黄边线到白线路口，并校正
	go_line(gd_m2);
	intersection_correct();
}

/**********************功能子函数**********************/

/****************************************
  函数名称:go_line
  参数:
  x 时间(单位:s)
  作用:沿黄线行走一段时间
  返回:/
  最后更新日期:2023/08/01
 *****************************************/
void go_line(float x) {
	float tt;

	tt = seconds(1) + x;
	while (seconds(1) < tt) {
		if (gd_fr1 < gdfr_m1) run_f(120, 250);
		else if (gd_fr2 < gdfr_m2) run_f(200, 250);
		else run_f(250, 120);

		if (x == gd_m2 && gd2 < gd_m2) break; //当gd2识别到白色，直接结束。不考虑时间。
	}
	run_f(0, 0);
	servo(sev_gd, s_gd_up);
}

/****************************************
  函数名称:go_fr_tp
  参数:
  x 时间(单位:s)
  作用:沿墙行走一段时间[PID控制][带平移]
  返回:/
  最后更新日期:2023/08/14
 *****************************************/
void go_fr_tp(float time) {
	int distance = 0;     //存储测得的距离
	double error = 0;     //当前误差
	double output = 0;    //PID 控制输出
	int target = 350;     //目标位置
	double prevError = 0; //上一次的误差
	int speed = 450;      //基础速度
	double integral = 0;  //累计误差
	double tt = seconds(1) + time;

	double KP = 1.20; //比例系数
	double KI = 0.02; //积分系数
	double KD = 0.30; //微分系数

	double KP_M = 0.3; //平移比例系数

	while (seconds(1) < tt) {
		distance = irr;// 读取传感器测得的距离
		error = target - distance;// 计算误差

		if (error > 150) { //误差过大，平移调整
			run(500, KP_M * error, 400, KP_M * error);
			wait(0.01);
			continue;
		}

		integral += error;// 更新累计误差

		if (abs(error) > 100) {
			integral = 0;
		}

		output = KP * error + KD * (error - prevError) + KI * integral;// 计算 PID 控制输出
		prevError = error;// 更新上一次的误差

		// 根据 PID 控制输出调整电机速度
		int LeftSpeed = speed + output;
		int RightSpeed = speed - output;

		run_f(LeftSpeed, RightSpeed);
		wait(0.01);
	}
	stop();
}

/****************************************
  函数名称:go_fr_t
  参数:
  x 时间(单位:s)
  作用:沿墙行走一段时间[PID控制]
  返回:/
  最后更新日期:2023/08/13
 *****************************************/

void go_fr_t(float time) {
	int distance = 0;     //存储测得的距离
	double error = 0;     //当前误差
	double output = 0;    //PID 控制输出
	int target = 350;     //目标位置
	double prevError = 0; //上一次的误差
	int speed = 450;      //基础速度
	double integral = 0;  //累计误差
	double tt = seconds(1) + time;

	double KP = 1.20; //比例系数
	double KI = 0.02; //积分系数
	double KD = 0.30; //微分系数

	while (seconds(1) < tt) {

		distance = irr; // 读取传感器测得的距离
		error = target - distance;// 计算误差
		integral += error;// 更新累计误差

		if (abs(error) > 100) {
			integral = 0;
		}

		output = KP * error + KD * (error - prevError) + KI * integral;// 计算 PID 控制输出
		prevError = error;// 更新上一次的误差

		// 根据 PID 控制输出调整电机速度
		int LeftSpeed = speed + output;
		int RightSpeed = speed - output;

		run_f(LeftSpeed, RightSpeed);
		wait(0.01);
	}
	stop();
}

/****************************************
  函数名称:go_fr_s
  参数:
  x 时间
  作用:沿右侧墙低速行进--使用在投放物块时
  返回:/
  最后更新日期:2023/08/08
 *****************************************/
void go_fr_s(float x) {
	float tt, ttt;

	tt = seconds(1) + x;
	ttt = seconds(1) + 0.1;
	while (true) {
		if (irr > irr_4) {
			run_f(150, 300);
		}
		else if (irr > irr_6) {
			run_f(300, 300);
		}
		else if (irr > irr_9) {
			run_f(300, 200);
		}
		else {
			run_f(300, 120);
		}

		if (seconds(1) > tt) {
			break;
		}
		if (x == 1000 && irr_cz < 350 && seconds(1) > ttt) {
			break;
		}
		else if (x > 100 && x < 1000 && irr < x && seconds(1) > ttt) {
			break;
		}
	}
	run_f(0, 0);
}

//前进到前方护栏 1n直行（建议偏左）前进，0n沿墙前进；  n1到墙停，n2到墙左转90度  n3到墙左转30度（用于高架第二平台）
void goto_barrier(int x) {
	while (irf < irf_8) {
		if (x < 10) {
			go_fr_t(0.01);
		}
		else {
			run_f(240, 250);
		}

		if (x == 3 && irf > 300) {
			break;
		}
	}
	run_f(0, 0);
	sound(500, 200);
	if (x == 2) {
		run_f(-350, 350);
		wait(0.38);
		run_f(0, 0);
		sound(500, 100);
	}
	else if (x == 3) {
		run_f(-350, 350);
		wait(0.15);
		go_fr_t(0.15);
	}
}


/****************************************
  函数名称:goto_intersection
  参数:/
  作用:前进到路口
  返回:/
  最后更新日期:2023/08/08
 *****************************************/
void goto_intersection(void) {
	while (gd2 > gd_m2 && gd3 > gd_m3) {
		if (irf > irf_brake) {
			run_f(0, 0);
			while (irf > (irf_brake - 20)) {
				wait(0.05);
			}
			wait(1);
		}
		else {
			go_fr_t(0.01);
		}
	}
	run_f(0, 0);
}

/****************************************
  函数名称:goto_intersection_new
  参数:/
  作用:前进到路口(new)
  返回:/
  最后更新日期:2023/08/11
 *****************************************/
void goto_intersection_new(void) {
	while (true) {
		if (gd2 < gd_m2 && (gd_m2 - gd2) > 40 && gd3 < gd_m3 && (gd_m3 - gd3) > 40) {
			break;
		}
		if (gd2 < gd_m2 && (gd_m2 - gd2) > 40 && irr < irr_null) {
			break;
		}
		if (gd3 < gd_m3 && (gd_m3 - gd3) > 40 && irr < irr_null) {
			break;
		}

		/*
		  1. (gd2 < gd_m2 && (gd_m2 - gd2)) > 40 && (gd3 < gd_m3 && (gd_m3 - gd3) > 40) 双白
		  2. (gd2 < gd_m2 && (gd_m2 - gd2) > 40 && irr < irr_null)  左白 && 右测距探测到路口
		  3. (gd3 < gd_m3 && (gd_m3 - gd3) > 40 && irr < irr_null)  右白 && 右测距探测到路口
		 */

		go_fr_t(0.01);
	}
	stop();
}

/****************************************
  函数名称:ground_test_input
  参数:/
  作用:从本地存储数据中读取地灰阈值并写入程序
  返回:/
  最后更新日期:2023/08/10
 *****************************************/
void ground_test_input(void) {
	gd_m1 = EE_re(0);
	gd_m2 = EE_re(1);
	gd_m3 = EE_re(2);
	gd_m4 = EE_re(3);
	gdfr_m1 = EE_re(4);
	gdfr_m2 = EE_re(5);
	gd_m3_b = EE_re(6);

	return;
}

/****************************************
  函数名称:ground
  参数:
  mode 模式
  1 读取之前数据
  2 测试数据
  作用:初始化地灰
  返回:/
  最后更新日期:2023/08/11
 *****************************************/
void ground(int mode) {
	if (mode == 1) { //读取之前的数据
		ground_test_input();
	}
	else { //重新检测
		ground_test();
		cls();
		sev_rst();
	}
	wait(0.3);
	return;
}

/****************************************
  函数名称:ground_test
  参数:/
  作用:地灰测试,并写入机器人
  返回:/
  最后更新日期:2023/08/11
 *****************************************/
void ground_test(void) {

	/***********
	  gd1 左地灰     [校正路口]
	  gd2 左前地灰   [检测路口]
	  gd3 右前地灰   [检测路口]
	  gd4 右地灰        [矫正路口]

	  gd_fr1 左黄线地灰
	  gd_fr2 右黄线地灰
	 ***********/

	//黑色
	int gd1_black;
	int gd2_black;
	int gd3_black;
	int gd4_black;
	int gdfr1_black;
	int gdfr2_black;

	//白色
	int gd1_white;
	int gd2_white;
	int gd3_white;
	int gd4_white;

	//黄色
	int gdfr1_yellow;
	int gdfr2_yellow;

	servo(1, s_gd_down); //放下移动地灰

	/******开始检测*******/

	//记录所有地灰黑色
	cls();
	wait(1);
	printf("Ground_Test\n1.gd1-4 gdfr black");
	while (!trigger()); //等待按下按钮
	//记录数值
	gd1_black = gd1;
	gd2_black = gd2;
	gd3_black = gd3;
	gd4_black = gd4;
	gdfr1_black = gd_fr1;
	gdfr2_black = gd_fr2;

	//记录前地灰白色
	cls();
	wait(1);
	printf("2.gd2-3 white");
	while (!trigger()); //等待按下按钮
	//记录数值
	gd2_white = gd2;
	gd3_white = gd3;

	//记录左右地灰白色
	cls();
	wait(1);
	printf("3.gd1,gd4 white");
	while (!trigger()); //等待按下按钮
	//记录数值
	gd1_white = gd1;
	gd4_white = gd4;

	//记录移动地灰黄色
	cls();
	wait(1);
	printf("4.gdfr1-2 yellow");
	while (!trigger()); //等待按下按钮
	//记录数值
	gdfr1_yellow = gd_fr1;
	gdfr2_yellow = gd_fr2;

	//记录右前地灰3悬空值
	cls();
	wait(1);
	printf("6.gd3 悬空");
	while (!trigger()); //等待按下按钮
	//记录数值
	gd_m3_b = gd3;

	/******结束检测*******/

	/******计算阈值*******/
	gd_m1 = (gd1_black + gd1_white) / 2;
	gd_m2 = (gd2_black + gd2_white) / 2;
	gd_m3 = (gd3_black + gd3_white) / 2;
	gd_m4 = (gd4_black + gd4_white) / 2;
	gdfr_m1 = (gdfr1_black + gdfr1_yellow) / 2;
	gdfr_m2 = (gdfr2_black + gdfr2_yellow) / 2;
	gd_m3_b -= 30;

	/*写入数据，便于下次使用*/
	EE_wr(0, gd_m1);
	EE_wr(1, gd_m2);
	EE_wr(2, gd_m3);
	EE_wr(3, gd_m4);
	EE_wr(4, gdfr_m1);
	EE_wr(5, gdfr_m2);
	EE_wr(6, gd_m3_b);

	/*提示OK*/
	cls();
	printf("OK!");
	sound(700, 300);
	cls();

	return;
}

/****************************************
  函数名称:intersection_right
  参数:
  degree 旋转角度
  作用:路口右转
  返回:/
  最后更新日期:2023/08/19
 *****************************************/
void intersection_right(int degree) {
	run_ft(500, 500, 0.33);
	turn_P(degree);
	run_ft(500, 500, 0.33);
}

/****************************************
  函数名称:intersection_left
  参数:
  degree 旋转角度
  作用:路口左转
  返回:/
  最后更新日期:2023/08/19
 *****************************************/
void intersection_left(int degree) {
	run_ft(500, 500, 0.74);
	turn_P(degree);
	run_ft(500, 500, 0.74);
}

void intersection_handling(int x)//路口处理(1普通路口，11/22/33转至黄线路口）   x=1直行   x=22左转    x=33右转
{
	float tt;

	if (x == 1 || x == 11) {
		run_f(300, 300);
		wait(0.1);
		run_f(392, 400);   //>>>小车过路口时建议左偏一点，避免撞到右侧墙（但不能压线）
		t_brake = seconds(1) + 1.25; //>>>控制小车过路口的时间
		Auto_brake(392, 400);
		if (x == 1 && irr < irr_12) {
			while (irr < irr_9) {
				run(300, 0, 300, 0);
			}
			run_f(0, 0);
		}
		else if (x == 11) {
			run_f(0, 0);
			servo(sev_gd, s_gd_down);
			sound(800, 300);
			while (gd_fr2 > gdfr_m2 && gd_fr1 > gdfr_m1) {
				run(250, -200, 250, -200);
			}
			run_f(0, 0);
			sound(800, 100);
		}
	}
	else if (x == 22) {
		run_f(300, 300);
		wait(0.1);
		run_f(400, 400);
		t_brake = seconds(1) + 0.55; //>>>控制小车直行到左转位（通过小车转弯后位置进行调整，转弯后小车整体过虚黄线）
		Auto_brake(400, 400);
		run_f(90, 400);
		t_brake = seconds(1) + 0.73; //>>>小车左转角度时间（建议80-85度）
		Auto_brake(90, 400);
		run_f(400, 400);
		t_brake = seconds(1) + 0.6; //>>>控制小车车头直行过路口
		Auto_brake(400, 400);
		run_f(0, 0);
		servo(sev_gd, s_gd_down);
		sound(800, 300);
		while (gd_fr2 > gdfr_m2) run(250, -200, 250, -200);
		run_f(0, 0);
		sound(800, 100);
	}
	else if (x == 33)
	{
		servo(sev_gd, s_gd_down);
		run_f(500, 198);
		wait(0.7);
		run_f(0, 0);
		sound(800, 100); //---调时间让小车右转过路口，小车平行于黄边线（不能碰掉头标志线）;
		while (gd_fr2 > gdfr_m2) run(250, -200, 250, -200);
		run_f(0, 0);
		sound(800, 100);
	}
	run_f(0, 0);
}

/****************************************
  函数名称:intersection_correct
  参数:/
  作用:路口矫正
  返回:/
  最后更新日期:2023/08/01
 *****************************************/
void intersection_correct(void) {
	while (gd2 < (gd_m2 + 20) || gd3 < (gd_m3 + 20)) {
		if (gd2 > gd_m2 && gd3 < gd_m3) run_f(-120, 200);
		else if (gd2 < gd_m2 && gd3 > gd_m3) run_f(200, -120);
		else run_f(180, 180);
	}
	while (gd1 > (gd_m1 - 20) || gd4 > (gd_m4 - 20)) {
		if (gd1 < gd_m1 && gd4 > gd_m4) run_f(-120, 200);
		else if (gd1 > gd_m1 && gd4 < gd_m4) run_f(200, -120);
		else run_f(180, 180);
	}
	run_f(0, 0);
	sound(500, 100); //sound(800,500);  wait(1);
}

/****************************************
  函数名称:intersection_turn
  参数:
  t1 t2 旋转角度
  作用:路口掉头
  返回:/
  最后更新日期:2023/08/19
 *****************************************/
void intersection_turn(int t1, int t2) {
	run_ft(500, 500, 0.16);
	turn_P(t1);
	run_ft(500, 500, 0.48);
	turn_P(t2);
	run_ft(500, 500, 0.2);
}

/****************************************
  函数名称:red_led_light
  参数:/
  作用:红绿灯路口检测
  返回:/
  最后更新日期:2023/08/01
 *****************************************/
void red_led_light(void) {
	int iii = 0;
	while (true) {
		if (Open_MV > 180 && Open_MV < 280) iii++;
		else iii = 0;
		if (iii > 3) break;
		stop();
		wait(0.02);
	}
}

/****************************************
  函数名称:sev_rst
  参数:/
  作用:舵机初始化
  返回:/
  最后更新日期:2023/09/03
 *****************************************/
void sev_rst(void) {
	servo(sev_gd, s_gd_up);
	wait(0.3);
	servo(sev_2, s_catch);
	wait(0.3);
	servo(sev_1, s_up);
	wait(0.3);
	servo(sev_blocks, s_back);
	wait(0.3);
	sound(1200, 200);
	cls();
	printf("按运行键启动");
	while (trigger() == 0) {
		wait(0.05);
	}
	cls();
}

/****************************************
  函数名称:sev_catch_up
  参数:
  x 物块类型
  1 大物块
  2 小物块
  作用:抓取物块
  返回:/
  最后更新日期:2023/08/01
 *****************************************/
void sev_catch_up(int x) {
	//前进到抓物块位置
	go_fr_s(1000);
	if (x == 1) {
		go_fr_s(0.05);
	}
	else if (x == 2) {
		go_fr_s(0.1);
	}
	run_f(0, 0);
	sound(800, 500);

	//抓物块
	servo(sev_1, s_down + 1000);
	wait(0.5);
	servo(sev_2, s_open);
	wait(0.5);
	servo(sev_1, s_down);
	wait(0.5);
	sound(800, 300);

	if (x == 1) {
		servo(sev_2, s_catch);
	}
	if (x == 2) {
		servo(sev_2, s_catch + 50);
	}

	wait(1);
	sound(800, 200);

	//收起机械臂
	servo(sev_1, s_up);
	wait(0.5);
}

/****************************************
  函数名称:sev_put_down
  参数:
  x 物块类型
  1 大物块
  2 小物块
  作用:放下物块
  返回:/
  最后更新日期:2023/08/01
 *****************************************/
void sev_put_down(int x)
{
	//靠近平台
	while (irr < irr_6) {
		run(200, -250, 200, -250);
	}   //---通过irr6测距
	run_f(0, 0);
	sound(800, 100);

	//投放物块--如多个都放开爪碰物块时可调整投放角度
	if (x == 1) {
		servo(sev_1, s_down + 100);
		wait(0.5);
		sound(800, 500);

		servo(sev_2, s_open - 400);
		wait(0.5);
		sound(800, 500);
	}
	else {
		servo(sev_1, s_down + 100);
		wait(0.5);

		servo(sev_1, s_down);
		wait(0.5);
		sound(800, 300);

		servo(sev_2, s_open);
		wait(0.5);
		sound(800, 500);
	}
	//收回爪子
	servo(sev_1, s_down + 1000);
	wait(0.5);
	servo(sev_2, s_catch);
	wait(0.5);
	servo(sev_1, s_up);
	wait(0.3);
	sound(800, 300);

	return;
}

void sev_gd_down(void) {
	servo(sev_gd, s_gd_down);
}

/****************************************
  函数名称:Auto_brake
  参数:
  x 左速度
  y 右速度
  作用:自动防撞
  返回:/
  最后更新日期:2023/08/01
 *****************************************/
void Auto_brake(int x, int y) {
	while (seconds(1) < t_brake) {
		if (irf > irf_brake) {
			stop();
			i_brake = 0;
			while (irf > (irf_brake - 20)) {
				wait(0.05);
				i_brake++;
			}
			wait(1);
			t_brake = t_brake + 1 + i_brake * 0.05 + 0.03;
		}
		else {
			run_f(x, y);
		}
	}
}

/****************************************
  函数名称:run_f
  参数:
  x 左速度
  y 右速度
  作用:直排轮控制模式运动
  返回:/
  最后更新日期:2023/08/01
 *****************************************/
void run_f(int x, int y) {
	run(x, x, y, y);
}

/****************************************
  函数名称:run
  参数:
  a m1速度
  b m2速度
  c m3速度
  d m4速度
  作用:四轮独立控制运动
  返回:/
  最后更新日期:2023/08/01
 *****************************************/
void run(int a, int b, int c, int d) { //4轮独立控制
	motor(1, a);
	motor(2, b);
	motor(3, c);
	motor(4, d);
}

/****************************************
  函数名称:run_ft
  参数:
  x 左(m1,m2)速度
  y 右(m3,m4)速度
  t 运动时间
  作用:直排轮模式运动计时
  返回:/
  最后更新日期:2023/08/08
 *****************************************/
void run_ft(int x, int y, double t) {
	run_f(x, y);
	wait(t);
	stop();
}

/****************************************
  函数名称:run_t
  参数:
  a m1速度
  b m2速度
  c m3速度
  d m4速度
  t 运动时间
  作用:麦克纳姆轮模式运动计时
  返回:/
  最后更新日期:2023/08/08
 *****************************************/
void run_t(int a, int b, int c, int d, double t) {
	run(a, b, c, d);
	wait(t);
	stop();
	return;
}

/****************************************
  函数名称:turn_P
  参数:
  targetAngle 目标旋转角度
  作用:P控制算法 旋转
  返回:/
  最后更新日期:2023/08/19
 *****************************************/
void turn_P(double targetAngle) {//P控制旋转

	double Kp = 7.0;
	targetAngle += cps;
	int angleError = targetAngle - cps; //角度差值

	int l, r; //左速度 右速度

	while (abs(angleError) > 2.0) {
		angleError = targetAngle - cps;

		// 修正角度差值
		if (angleError > 180) {
			angleError -= 360;
		}
		else if (angleError < -180) {
			angleError += 360;
		}

		l = Kp * angleError;
		r = -Kp * angleError;

		if (abs(l) < 250) { //避免速度太小导致无法运动
			l = abs(l) / l * 250;
			r = -l;
		}

		run_f(l, r);
	}

	stop();
}





/****************************************
  函数名称:stop
  参数:/
  作用:机器人停止运动
  返回:/
  最后更新日期:2023/08/08
 *****************************************/
void stop(void) {
	run_f(0, 0);
	return;
}



/****************************************
  函数名称:EE_wr
  参数:
  a 数据编号
  b 数据内容
  作用:写入数据
  返回:/
  最后更新日期:2023/08/10
 *****************************************/
void EE_wr(int a, int b) {
	EE_WriteVariable(a, b);

	return;
}

/****************************************
  函数名称:EE_re
  参数:
  a 数据编号
  作用:读取存储的数据
  返回:读取的数据(int)
  最后更新日期:2023/08/10
 *****************************************/
int EE_re(int a) {
	return EE_ReadVariable(a);
}

void test(int n) {
	switch (n) {
		case 1:
			while (irr_cz > 300) {
				go_fr_tp(0.01);
			}
			while (irr_cz < 300) {
				run_f(200, 200);
			}
			stop();

			sound(1000, 300);

			servo(sev_1, s_down);
			wait(0.2);
			servo(sev_2, s_open);
			wait(0.6);
			servo(sev_2, s_catch - 400);
			sound(1200, 400);
			servo(sev_1, s_up);
			wait(1);
			servo(sev_2, s_open - 200);
			wait(0.5);
			servo(sev_2, s_catch - 150);

			sound(1500, 300);

			break;
		default:
			break;
	}
	return;
}

void catch_blocks(int m) {

}
