#include "PX4CtrlFSM.h"
#include <uav_utils/converters.h>
#include <cmath>
#include <mavros_msgs/VehicleThrustAccSetpoint.h>
#include <mavros_msgs/OffboardControlMode.h>
using namespace std;
using namespace uav_utils;

PX4CtrlFSM::PX4CtrlFSM(Parameter_t &param_, Controller &controller_) : param(param_), controller(controller_) /*, thrust_curve(thrust_curve_)*/
{
	state = MANUAL_CTRL;
	hover_pose.setZero();
	cmdctrl_reentry_forbidden_latched = false;
	cmdctrl_acc_setpoint_published_once = false;
	rc_stick_abort_offboard_latched = false;
	last_altctl_request_time = ros::Time(0);
}

/* 
        Finite State Machine

	      system start
	            |
	            |
	            v
	----- > MANUAL_CTRL <-----------------
	|         ^   |    \                 |
	|         |   |     \                |
	|         |   |      > AUTO_TAKEOFF  |
	|         |   |        /             |
	|         |   |       /              |
	|         |   |      /               |
	|         |   v     /                |
	|       AUTO_HOVER <                 |
	|         ^   |  \  \                |
	|         |   |   \  \               |
	|         |	  |    > AUTO_LAND -------
	|         |   |
	|         |   v
	-------- CMD_CTRL

*/

void PX4CtrlFSM::process()
{

	ros::Time now_time = ros::Time::now();
	// 发给px4的u
	Controller_Output_t u;
	Desired_State_t des(odom_data);//保证默认的des是当前odom
	Desired_at_w_t des_at_W;des_at_W.ft=0;des_at_W.wx=0;des_at_W.wy=0;des_at_W.wz=0;
	bool rotor_low_speed_during_land = false;
	// cout << "state: " << state << endl;
	// STEP1: state machine runs
	switch (state)
	{
	case MANUAL_CTRL:
	{
		
		if (rc_data.enter_hover_mode) // Try to jump to AUTO_HOVER
		{
			if (!odom_is_received(now_time))
			{
				ROS_ERROR_THROTTLE(1.0, "[px4ctrl] Reject AUTO_HOVER(L2). No odom!");
				break;
			}
			if (cmd_is_received(now_time))
			{
				ROS_ERROR_THROTTLE(1.0, "[px4ctrl] Reject AUTO_HOVER(L2). You are sending commands before toggling into AUTO_HOVER, which is not allowed. Stop sending commands now!");
				break;
			}
			if (odom_data.v.norm() > 3.0)
			{
				ROS_ERROR_THROTTLE(1.0, "[px4ctrl] Reject AUTO_HOVER(L2). Odom_Vel=%fm/s, which seems that the locolization module goes wrong!", odom_data.v.norm());
				break;
			}

			state = AUTO_HOVER;
			controller.resetThrustMapping();
			set_hov_with_odom();
			toggle_offboard_mode(true);//offboard

			ROS_INFO_THROTTLE(1.0, "\033[32m[px4ctrl] MANUAL_CTRL(L1) --> AUTO_HOVER(L2)\033[32m");
		}
		else if (param.takeoff_land.enable && takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::TAKEOFF) // Try to jump to AUTO_TAKEOFF
		{
			if (!odom_is_received(now_time))
			{
				ROS_ERROR_THROTTLE(1.0, "[px4ctrl] Reject AUTO_TAKEOFF. No odom!");
				break;
			}
			if (cmd_is_received(now_time))
			{
				ROS_ERROR_THROTTLE(1.0, "[px4ctrl] Reject AUTO_TAKEOFF. You are sending commands before toggling into AUTO_TAKEOFF, which is not allowed. Stop sending commands now!");
				break;
			}
			if (odom_data.v.norm() > 0.1)
			{
				ROS_ERROR_THROTTLE(1.0, "[px4ctrl] Reject AUTO_TAKEOFF. Odom_Vel=%fm/s, non-static takeoff is not allowed!", odom_data.v.norm());
				break;
			}
			if (!get_landed())
			{
				ROS_ERROR_THROTTLE(1.0, "[px4ctrl] Reject AUTO_TAKEOFF. land detector says that the drone is not landed now!");
				break;
			}
			if (rc_is_received(now_time)) // Check this only if RC is connected.
			{
				if (!rc_data.is_hover_mode || !rc_data.is_command_mode || !rc_data.check_centered())
				{
					ROS_ERROR_THROTTLE(1.0, "[px4ctrl] Reject AUTO_TAKEOFF. If you have your RC connected, keep its switches at \"auto hover\" and \"command control\" states, and all sticks at the center, then takeoff again.");
					while (ros::ok())
					{
						ros::Duration(0.01).sleep();
						ros::spinOnce();
						if (rc_data.is_hover_mode && rc_data.is_command_mode && rc_data.check_centered())
						{
							ROS_INFO_THROTTLE(1.0, "\033[32m[px4ctrl] OK, you can takeoff again.\033[32m");
							break;
						}
					}
					break;
				}
			}

			state = AUTO_TAKEOFF;
			
			controller.resetThrustMapping();
			set_start_pose_for_takeoff_land(odom_data);
			toggle_offboard_mode(true);				  // toggle on offboard before arm
			for (int i = 0; i < 10 && ros::ok(); ++i) // wait for 0.1 seconds to allow mode change by FMU // mark
			{
				ros::Duration(0.01).sleep();
				ros::spinOnce();
			}
			if (param.takeoff_land.enable_auto_arm)
			{
				toggle_arm_disarm(true);
			}
			takeoff_land.toggle_takeoff_land_time = now_time;

			ROS_INFO_THROTTLE(1.0, "\033[32m[px4ctrl] MANUAL_CTRL(L1) --> AUTO_TAKEOFF\033[32m");
		}
		else if(rc_data.positon_manual_offboard_mode==2){
			//MANUAL_CTRL
			toggle_offboard_mode(false);
			ROS_INFO_THROTTLE(1.0, "\033[32m[px4ctrl]  positon_manual_offboard_mode=2  but position_ctrl\033[32m");
		}
		else if(rc_data.positon_manual_offboard_mode==1){
			//MANUAL_CTRL
			toggle_offboard_mode(false);
			ROS_INFO_THROTTLE(1.0, "\033[32m[px4ctrl]  position_ctrl\033[32m");
		}
		else if(rc_data.positon_manual_offboard_mode==0){
			//MANUAL_CTRL
			toggle_offboard_mode(false);
			ROS_INFO_THROTTLE(1.0, "\033[32m[px4ctrl] MANUAL_CTRL(L1) \033[32m");
		}
		if (rc_data.toggle_reboot) // Try to reboot. EKF2 based PX4 FCU requires reboot when its state estimator goes wrong.
		{
			if (state_data.current_state.armed)
			{
				ROS_ERROR_THROTTLE(1.0, "[px4ctrl] Reject reboot! Disarm the drone first!");
				break;
			}
			reboot_FCU();
		}

		break;
	}

	case AUTO_HOVER:
	{
		const bool recovery_latched =
			(param.forbid_cmdctrl_reentry_after_loss &&
			 cmdctrl_reentry_forbidden_latched) ||
			rc_stick_abort_offboard_latched;

		if (!rc_data.is_hover_mode || !odom_is_received(now_time))
		{
			request_altctl_mode(now_time, "From AUTO_HOVER(L3) to AUTO_HOVER(L2)!-----call ALTCTL");
			ROS_INFO_THROTTLE(1.0,"[px4ctrl] From AUTO_HOVER(L2) to AUTO_HOVER(L2)!");
			

		}
		else if (rc_data.is_command_mode && cmd_is_received(now_time))   //cmd模式 经过更改后应该属于mpc
		{
			const bool reentry_blocked =
				(param.forbid_cmdctrl_reentry_after_loss &&
				 cmdctrl_reentry_forbidden_latched) ||
				rc_stick_abort_offboard_latched;
			if (!reentry_blocked && state_data.current_state.mode == "OFFBOARD")
			{
				state = CMD_CTRL;
				// des = get_cmd_des();
				des_at_W= get_cmd_des();
				ROS_INFO_THROTTLE(1.0,"\033[32m[px4ctrl] AUTO_HOVER(L2) --> CMD_CTRL(L3)\033[32m");
				start_pose_data.recv_new_msg=false;
				start_pose_data.reached_start_pose=true;
				start_pose_data.getmsg_want_toggle=false;
				ROS_INFO_THROTTLE(1.0,"start_pose_data.reached_start_pose 1  start_pose_data.recv_new_msg 0");
			}
			else if (reentry_blocked)
			{
				ROS_INFO_THROTTLE(1.0, "[px4ctrl] CMD_CTRL re-entry blocked after one loss. Restart node to re-enable.");
			}
		}
		else if (takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::LAND)
		{

			state = AUTO_LAND;
			set_start_pose_for_takeoff_land(odom_data);

			ROS_INFO_THROTTLE(1.0,"\033[32m[px4ctrl] AUTO_HOVER(L2) --> AUTO_LAND\033[32m");
		}
		else if((!recovery_latched) && start_pose_data.recv_new_msg && (! start_pose_data.reached_start_pose))
		{//当心不要从cmd指令消失达到hover的时候去hover这个了
			des =get_hover_des_with_planner_start_pose();
			set_hov_with_odom();
			if(start_pose_data.getmsg_want_toggle==true){
				toggle_offboard_mode(true);//这样一直call这个服务好吗
				ROS_INFO_THROTTLE(1.0,"toggle_offboard_mode(true)  in   AUTO_HOVER");
				start_pose_data.getmsg_want_toggle=false;
			}
			ROS_INFO_THROTTLE(1.0,"\033[32m[px4ctrl]  get_hover_des_with_planner_start_pose()  \033[32m");
		}
		else//前后左右 位置模式
		{
			// set_hov_with_rc();
			// des = get_hover_des();
			// // std::cout<<"des"<<std::endl;
			// if ((rc_data.enter_command_mode) ||
			// 	(takeoff_land.delay_trigger.first && now_time > takeoff_land.delay_trigger.second))
			// {
			// 	takeoff_land.delay_trigger.first = false;
			// 	publish_trigger(odom_data.msg);
			// 	ROS_INFO("\033[32m[px4ctrl] TRIGGER sent, allow user command.\033[32m");
			// }
			// std::cout << "[ms=" << (now_time.toNSec() / 1000000ULL) << "] debug state_data.current_state.mode " << state_data.current_state.mode << std::endl;
			request_altctl_mode(now_time, "--------------------call ALTCTL");
		}

		break;
	}

	case CMD_CTRL:
	{
// 		if (!rc_data.is_hover_mode || !odom_is_received(now_time))
// 		{
// 			state = MANUAL_CTRL;
// 			toggle_offboard_mode(false);
// //先检查遥控5通modeset是maunal posctl offboard
// //如果被切走那就按maunal posctl算
// 			ROS_WARN("[px4ctrl] From CMD_CTRL(L3) to MANUAL_CTRL(L1)!");
// 		}
// 		else if (!rc_data.is_command_mode || !cmd_is_received(now_time))
// 		{

		if (!rc_data.is_command_mode || !cmd_is_received(now_time)||!rc_data.is_hover_mode || !odom_is_received(now_time))
		{
			if (param.forbid_cmdctrl_reentry_after_loss)
			{
				cmdctrl_reentry_forbidden_latched = true;
				ROS_WARN("[px4ctrl] CMD_CTRL exited once, re-entry is now forbidden until node restart.");
			}
			state = AUTO_HOVER;
			// set_hov_with_odom();
			// des = get_hover_des();

//再检查遥控6通有没有紧急切到悬停  或者左边5通又没有切 右边
//如果被切走或者突然没指令那就按悬停算  

			request_altctl_mode(now_time, "From CMD_CTRL(L3) to AUTO_HOVER(L2)!-----call ALTCTL");
			ROS_INFO_THROTTLE(1.0,"[px4ctrl] From CMD_CTRL(L3) to AUTO_HOVER(L2)!");
			
//我想改成一拨就是posctl

		}
		else //继续cmdctrl
		{
			// des = get_cmd_des();
			// toggle_offboard_mode(true);
			des_at_W= get_cmd_des();
		}

		if (takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::LAND)
		{
			ROS_ERROR_THROTTLE(1.0, "[px4ctrl] Reject AUTO_LAND, which must be triggered in AUTO_HOVER. \
					Stop sending control commands for longer than %fs to let px4ctrl return to AUTO_HOVER first.",
					  param.msg_timeout.cmd);
		}

		break;
	}

	case AUTO_TAKEOFF:
	{
		if ((now_time - takeoff_land.toggle_takeoff_land_time).toSec() < AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME) // Wait for several seconds to warn prople.
		{
			des = get_rotor_speed_up_des(now_time);
		}
		else if (odom_data.p(2) >= (takeoff_land.start_pose(2) + param.takeoff_land.height)) // reach the desired height
		{
			state = AUTO_HOVER;
			set_hov_with_odom();
			ROS_INFO_THROTTLE(1.0, "\033[32m[px4ctrl] AUTO_TAKEOFF --> AUTO_HOVER(L2)\033[32m");

			takeoff_land.delay_trigger.first = true;
			takeoff_land.delay_trigger.second = now_time + ros::Duration(AutoTakeoffLand_t::DELAY_TRIGGER_TIME);
		}
		else
		{
			des = get_takeoff_land_des(param.takeoff_land.speed);
		}

		break;
	}

	case AUTO_LAND:
	{
		if (!rc_data.is_hover_mode || !odom_is_received(now_time))
		{
			state = MANUAL_CTRL;
			toggle_offboard_mode(false);

			ROS_WARN("[px4ctrl] From AUTO_LAND to MANUAL_CTRL(L1)!");
		}
		else if (!rc_data.is_command_mode)
		{
			state = AUTO_HOVER;
			set_hov_with_odom();
			des = get_hover_des();
			ROS_INFO_THROTTLE(1.0, "[px4ctrl] From AUTO_LAND to AUTO_HOVER(L2)!");
		}
		else if (!get_landed())
		{
			des = get_takeoff_land_des(-param.takeoff_land.speed);
		}
		else
		{
			rotor_low_speed_during_land = true;

			static bool print_once_flag = true;
			if (print_once_flag)
			{
				ROS_INFO_THROTTLE(1.0, "\033[32m[px4ctrl] Wait for abount 10s to let the drone arm.\033[32m");
				print_once_flag = false;
			}

			if (extended_state_data.current_extended_state.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) // PX4 allows disarm after this
			{
				static double last_trial_time = 0; // Avoid too frequent calls
				if (now_time.toSec() - last_trial_time > 1.0)
				{
					if (toggle_arm_disarm(false)) // disarm
					{
						print_once_flag = true;
						state = MANUAL_CTRL;
						toggle_offboard_mode(false); // toggle off offboard after disarm
						ROS_INFO_THROTTLE(1.0, "\033[32m[px4ctrl] AUTO_LAND --> MANUAL_CTRL(L1)\033[32m");
					}

					last_trial_time = now_time.toSec();
				}
			}
		}

		break;
	}

	default:
		break;
	}

	// STEP2: solve and update new control commands
	//按照原本的 是不管auto什么的或者cmdctrl的都会有一个des(pvaj),
	//然后统一形式进入solver计算 u 然后统一发布bodyrate或者姿态ctrl指令
	//而改mpcctrl  使cmdctrl这里get的des是des(at_w)
	if (rotor_low_speed_during_land) // used at the start of auto takeoff
	{
		motors_idling(imu_data, u);
	}
	else
	{
		switch (param.pose_solver)
		{
		case 0:
			debug_msg = controller.update_alg0(des, odom_data, imu_data, u, bat_data.volt);
			debug_msg.header.stamp = now_time;
			debug_pub.publish(debug_msg);
			break;
		case 1:
			// 这个函数就是将 要下传的函数 保存的 u 
			debug_msg = controller.update_alg1(des, odom_data, imu_data, u, bat_data.volt);
			debug_msg.header.stamp = now_time;
			debug_pub.publish(debug_msg);
			break;

		case 2:
			controller.update_alg2(des, odom_data, imu_data, u, bat_data.volt);
			break;
		case 3:
			debug_msg = controller.update_alg3(des, odom_data, imu_data, u, bat_data.volt);
			debug_msg.header.stamp = now_time;
			debug_pub.publish(debug_msg);
			break;
			
		default:
			ROS_ERROR_THROTTLE(1.0, "Illegal pose_slover selection!");
			return;
		}
	}

	// // STEP3: estimate thrust model
	// if (state == AUTO_HOVER || state == CMD_CTRL)
	// {
	// 	controller.estimateThrustModel(imu_data.a, bat_data.volt, odom_data.v, param);
	// }
	// // STEP4: publish control commands to mavros
	// if (param.use_bodyrate_ctrl)
	// {
	// 	publish_bodyrate_ctrl(u, now_time);
	// }
	// else
	// {
	// 	publish_attitude_ctrl(u, now_time);
	// }


	// STEP3: estimate thrust model  我不知道这里是不是要估推力模型  mpcctrl暂时不管
	if (state == AUTO_HOVER || state == CMD_CTRL)
	{
		controller.estimateThrustModel(imu_data.a, bat_data.volt, odom_data.v, param);
	}

	// 四通道急停：偏离中位则锁存并停止发 offboard 设定点（本帧起不再发 attitude/acc offboard）。
	// 与“拨杆后 0.4s 才进 ALTCTL、期间仍有 offboard”不同：此处主动停发后，飞控侧通常仍保持上一拍设定点直至 COM_OF_LOSS_T 超时再切模态，并非必然进入 ACRO/电机停转；具体以固件与实机为准。
	// apply_rc_stick_abort_offboard_if_needed 在 AUTO_LAND 时恒为 false（不阻塞发布、不在此阶段锁存/切状态），保证 land.sh 始终能发 attitude。
	const bool block_offboard_for_rc_stick = apply_rc_stick_abort_offboard_if_needed(now_time);

	// STEP4: publish control commands to mavros
	// 常规配置：ctrl_param 中 use_bodyrate_ctrl: false 且 use_normal_or_acc_closeloop: false。
	// takeoff / AUTO_HOVER / start_pose 等走 controller+publish_attitude_ctrl；上层 atw 走 publish_acc_closeloop_ctrl，均属 offboard 类设定点。
	// 若 use_bodyrate_ctrl==true 或 use_normal_or_acc_closeloop==true：下列分支已弃用（未与 RC 四通道急停联调），请勿在生产配置中开启。
	if (param.use_bodyrate_ctrl)
	{
		// 已弃用：请保持 use_bodyrate_ctrl=false；否则与 RC 四通道急停、acc 闭环分支未联调。
		if (!block_offboard_for_rc_stick)
		{
			if (state == CMD_CTRL)
			{
				if (param.use_normal_or_acc_closeloop)
				{
					// 已弃用：请保持 use_normal_or_acc_closeloop=false
					publish_at_w_from_mpc(des_at_W, now_time);
					ROS_ERROR_THROTTLE(1.0, "publish_at_w_from_mpc!");
				}
				else
				{
					// 已弃用：与默认 acc 闭环路径重复时请用下方 else 分支
					publish_offboard_mode();
					publish_acc_closeloop_ctrl(des_at_W, now_time);
					cmdctrl_acc_setpoint_published_once = true;
					ROS_ERROR_THROTTLE(1.0, "publish_at_w_from_mpc!  acc_closeloop");
				}
			}
			else
			{
				publish_bodyrate_ctrl(u, now_time);
			}
		}
	}
	else
	{
		if (block_offboard_for_rc_stick)
		{
			ROS_WARN_THROTTLE(2.0, "[px4ctrl] RC stick abort: skip publish_attitude_ctrl / publish_acc_closeloop_ctrl.");
		}
		else if (state == CMD_CTRL)
		{
			if (param.use_normal_or_acc_closeloop)
			{
				// 已弃用：请保持 use_normal_or_acc_closeloop=false（本机用 thrust+body_rate 的 mavlink 路径）
				publish_at_w_from_mpc(des_at_W, now_time);
				ROS_ERROR_THROTTLE(1.0, "publish_at_w_from_mpc!");
			}
			else
			{
				publish_offboard_mode();
				publish_acc_closeloop_ctrl(des_at_W, now_time);
				cmdctrl_acc_setpoint_published_once = true;
				ROS_ERROR_THROTTLE(1.0, "publish_at_w_from_mpc!  acc_closeloop");
			}
		}
		else
		{
			if (cmdctrl_acc_setpoint_published_once && state == AUTO_HOVER)
			{
				ROS_WARN_THROTTLE(1.0, "[px4ctrl] Skip publish_attitude_ctrl in AUTO_HOVER after CMD_CTRL acc-closeloop takeover.");
			}
			else
			{
				// 注意：此处是 state != CMD_CTRL（如 AUTO_HOVER / AUTO_TAKEOFF），不是「在 CMD_CTRL 里」
				ROS_INFO_THROTTLE(1.0, "[px4ctrl] publish_attitude_ctrl (state!=CMD_CTRL, e.g. AUTO_HOVER)");
				publish_attitude_ctrl(u, now_time);
			}
		}
	}


	// STEP5: Detect if the drone has landed
	land_detector(state, des, odom_data);
	// cout << takeoff_land.landed << " ";
	// fflush(stdout);

	// STEP6: Clear flags beyound their lifetime
	rc_data.enter_hover_mode = false;
	rc_data.enter_command_mode = false;
	rc_data.toggle_reboot = false;
	takeoff_land_data.triggered = false;
}

void PX4CtrlFSM::motors_idling(const Imu_Data_t &imu, Controller_Output_t &u)
{
	u.q = imu.q;
	// cout << u.q.w() << ',' << u.q.x()<< ',' << u.q.y()<< ',' << u.q.z() << endl;
	u.bodyrates = Eigen::Vector3d::Zero();
	u.thrust = 0.04;
}

void PX4CtrlFSM::land_detector(const State_t state, const Desired_State_t &des, const Odom_Data_t &odom)
{
	static State_t last_state = State_t::MANUAL_CTRL;
	if (last_state == State_t::MANUAL_CTRL && (state == State_t::AUTO_HOVER || state == State_t::AUTO_TAKEOFF))
	{
		takeoff_land.landed = false; // Always holds
	}
	last_state = state;

	if (state == State_t::MANUAL_CTRL && !state_data.current_state.armed)
	{
		takeoff_land.landed = true;
		return; // No need of other decisions
	}

	// land_detector parameters
	constexpr double POSITION_DEVIATION_C = -0.5; // Constraint 1: target position below real position for POSITION_DEVIATION_C meters.
	constexpr double VELOCITY_THR_C = 0.1;		  // Constraint 2: velocity below VELOCITY_MIN_C m/s.
	constexpr double TIME_KEEP_C = 3.0;			  // Constraint 3: Time(s) the Constraint 1&2 need to keep.

	static ros::Time time_C12_reached; // time_Constraints12_reached
	static bool is_last_C12_satisfy;
	if (takeoff_land.landed)
	{
		time_C12_reached = ros::Time::now();
		is_last_C12_satisfy = false;
	}
	else
	{
		bool C12_satisfy = (des.p(2) - odom.p(2)) < POSITION_DEVIATION_C && odom.v.norm() < VELOCITY_THR_C;
		if (C12_satisfy && !is_last_C12_satisfy)
		{
			time_C12_reached = ros::Time::now();
		}
		else if (C12_satisfy && is_last_C12_satisfy)
		{
			if ((ros::Time::now() - time_C12_reached).toSec() > TIME_KEEP_C) //Constraint 3 reached
			{
				takeoff_land.landed = true;
			}
		}

		is_last_C12_satisfy = C12_satisfy;
	}
}

Desired_State_t PX4CtrlFSM::get_hover_des()
{
	Desired_State_t des;
	des.p = hover_pose.head<3>();
	des.v = Eigen::Vector3d::Zero();
	des.a = Eigen::Vector3d::Zero();
	des.j = Eigen::Vector3d::Zero();
	des.yaw = hover_pose(3);
	des.yaw_rate = 0.0;

	return des;
}

// Desired_State_t PX4CtrlFSM::get_cmd_des()
// {
// 	Desired_State_t des;
// 	des.p = cmd_data.p;
// 	des.v = cmd_data.v;
// 	des.a = cmd_data.a;
// 	des.j = cmd_data.j;
// 	des.yaw = cmd_data.yaw;
// 	des.yaw_rate = cmd_data.yaw_rate;

// 	return des;
// }

Desired_at_w_t PX4CtrlFSM::get_cmd_des()
{
	Desired_at_w_t refer_at_W;


	if (param.use_normal_or_acc_closeloop){

		if (cmd_data.ft > 70) {
			ROS_WARN("WARN: MAX_THRUST");
			refer_at_W.ft = 1;//这里可能还得再限制限制
		}
		if (cmd_data.ft < 0.0) {
			ROS_WARN("WARN: MIN_THRUST");
			refer_at_W.ft = 0.0;
		}
		refer_at_W.ft = controller.get_thrust_0_1_from_ref_ft(cmd_data.ft);//(param.thr_map.hover_percentage/param.gra)*(cmd_data.ft/param.mass) ;//0.4-0.5
		refer_at_W.wx = cmd_data.wx;
		refer_at_W.wy = cmd_data.wy;
		refer_at_W.wz = cmd_data.wz;

	}
	else{
	refer_at_W.ft =  cmd_data.ft/param.mass;//acc
	refer_at_W.wx = cmd_data.wx;
	refer_at_W.wy = cmd_data.wy;
	refer_at_W.wz = cmd_data.wz;
	}

	return refer_at_W;
}

Desired_State_t PX4CtrlFSM::get_rotor_speed_up_des(const ros::Time now)
{
	double delta_t = (now - takeoff_land.toggle_takeoff_land_time).toSec();
	double des_a_z = exp((delta_t - AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME) * 6.0) * 7.0 - 7.0; // Parameters 6.0 and 7.0 are just heuristic values which result in a saticfactory curve.
	if (des_a_z > 0.1)
	{
		ROS_ERROR_THROTTLE(1.0, "des_a_z > 0.1!, des_a_z=%f", des_a_z);
		des_a_z = 0.0;
	}

	Desired_State_t des;
	des.p = takeoff_land.start_pose.head<3>();
	des.v = Eigen::Vector3d::Zero();
	des.a = Eigen::Vector3d(0, 0, des_a_z);
	des.j = Eigen::Vector3d::Zero();
	des.yaw = takeoff_land.start_pose(3);
	des.yaw_rate = 0.0;

	return des;
}

Desired_State_t PX4CtrlFSM::get_takeoff_land_des(const double speed)
{
	ros::Time now = ros::Time::now();
	double delta_t = (now - takeoff_land.toggle_takeoff_land_time).toSec() - (speed > 0 ? AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME : 0); // speed > 0 means takeoff
	// takeoff_land.last_set_cmd_time = now;

	// takeoff_land.start_pose(2) += speed * delta_t;

	Desired_State_t des;
	des.p = takeoff_land.start_pose.head<3>() + Eigen::Vector3d(0, 0, speed * delta_t);
	des.v = Eigen::Vector3d(0, 0, speed);
	des.a = Eigen::Vector3d::Zero();
	des.j = Eigen::Vector3d::Zero();
	des.yaw = takeoff_land.start_pose(3);
	des.yaw_rate = 0.0;

	return des;
}

void PX4CtrlFSM::set_hov_with_odom()
{
	hover_pose.head<3>() = odom_data.p;
	hover_pose(3) = get_yaw_from_quaternion(odom_data.q);

	last_set_hover_pose_time = ros::Time::now();
}

void PX4CtrlFSM::set_hov_with_rc()
{
	ros::Time now = ros::Time::now();
	double delta_t = (now - last_set_hover_pose_time).toSec();
	last_set_hover_pose_time = now;

	if (param.rc_mode.world_frame_ctrl)
	{
		// raw rc control the v in world frame
		hover_pose(0) += rc_data.ch[1] * param.max_manual_vel * delta_t * (param.rc_reverse.pitch ? 1 : -1);
		hover_pose(1) += rc_data.ch[0] * param.max_manual_vel * delta_t * (param.rc_reverse.roll ? 1 : -1);
		hover_pose(2) += rc_data.ch[2] * param.max_manual_vel * delta_t * (param.rc_reverse.throttle ? 1 : -1);
		hover_pose(3) += rc_data.ch[3] * param.max_manual_vel * delta_t * (param.rc_reverse.yaw ? 1 : -1);
		// cout << "ch[0~3]=" << rc_data.ch[0] << " " << rc_data.ch[1] << " " << rc_data.ch[2] << " " << rc_data.ch[3] << endl;
	}
	else{
		// jcx rc control the v in body frame
		hover_pose(3) += rc_data.ch[3] * param.max_manual_vel * delta_t * (param.rc_reverse.yaw ? 1 : -1);
		double yaw_cur = get_yaw_from_quaternion(odom_data.q);
		double vx_world, vy_world, vz_world;
		vx_world = cos(yaw_cur) * rc_data.ch[1]*(param.rc_reverse.pitch ? 1 : -1) - sin(yaw_cur) * rc_data.ch[0]* (param.rc_reverse.roll ? 1 : -1);
		vy_world = sin(yaw_cur) * rc_data.ch[1]*(param.rc_reverse.pitch ? 1 : -1) + cos(yaw_cur) * rc_data.ch[0]* (param.rc_reverse.roll ? 1 : -1);
		vz_world = rc_data.ch[2] * (param.rc_reverse.throttle ? 1 : -1);
		hover_pose(0) += vx_world * delta_t * param.max_manual_vel ;
		hover_pose(1) += vy_world * delta_t * param.max_manual_vel ;
		hover_pose(2) += vz_world * delta_t * param.max_manual_vel ;
		// cout << "yaw=" << yaw_cur << endl;
		// cout << "hover_pose(3)" << hover_pose(3) << endl;
		// cout << "ch[0~3]=" << rc_data.ch[0] << " " << rc_data.ch[1] << " " << rc_data.ch[2] << " " << rc_data.ch[3] << endl;
		// cout << "hover_pose=" << hover_pose(0) << " " << hover_pose(1) << " " << hover_pose(2) << endl;
	}



	if (hover_pose(2) < -0.3)
		hover_pose(2) = -0.3;

	// if (param.print_dbg)
	// {
	// 	static unsigned int count = 0;
	// 	if (count++ % 100 == 0)
	// 	{
	// 		cout << "hover_pose=" << hover_pose.transpose() << endl;
	// 		cout << "ch[0~3]=" << rc_data.ch[0] << " " << rc_data.ch[1] << " " << rc_data.ch[2] << " " << rc_data.ch[3] << endl;
	// 	}
	// }
}

Desired_State_t PX4CtrlFSM::get_hover_des_with_planner_start_pose()
{
	Desired_State_t des;
	des.p =  start_pose_data.p;
	des.v = Eigen::Vector3d::Zero();
	des.a = Eigen::Vector3d::Zero();
	des.j = Eigen::Vector3d::Zero();
	des.yaw = get_yaw_from_quaternion(start_pose_data.q);
	des.yaw_rate = 0.0;
	// start_pose_data.recv_new_msg=false;
	double yaw_cur = get_yaw_from_quaternion(odom_data.q);
	// std::cout<<"des.p "<<des.p<<"  odom_data.p  "<<odom_data.p<<std::endl;
	// std::cout<<"des.yaw "<<des.yaw<<"  yaw_cur  "<<yaw_cur<<std::endl;
	// if((des.p-odom_data.p).norm()<0.11 and abs(yaw_cur-des.yaw)<0.1)
	// {
	// 	start_pose_data.recv_new_msg=false;
	// 	start_pose_data.reached_start_pose=true;
	// }
		

	return des;
}

void PX4CtrlFSM::set_start_pose_for_takeoff_land(const Odom_Data_t &odom)
{
	takeoff_land.start_pose.head<3>() = odom_data.p;
	takeoff_land.start_pose(3) = get_yaw_from_quaternion(odom_data.q);

	takeoff_land.toggle_takeoff_land_time = ros::Time::now();
}

bool PX4CtrlFSM::rc_is_received(const ros::Time &now_time)
{
	return (now_time - rc_data.rcv_stamp).toSec() < param.msg_timeout.rc;
}

bool PX4CtrlFSM::cmd_is_received(const ros::Time &now_time)
{
	return (now_time - cmd_data.rcv_stamp).toSec() < param.msg_timeout.cmd;
}

bool PX4CtrlFSM::odom_is_received(const ros::Time &now_time)
{	
	return (now_time - odom_data.rcv_stamp).toSec() < param.msg_timeout.odom;
}

bool PX4CtrlFSM::imu_is_received(const ros::Time &now_time)
{
	return (now_time - imu_data.rcv_stamp).toSec() < param.msg_timeout.imu;
}

bool PX4CtrlFSM::bat_is_received(const ros::Time &now_time)
{
	return (now_time - bat_data.rcv_stamp).toSec() < param.msg_timeout.bat;
}

bool PX4CtrlFSM::recv_new_odom()
{
	if (odom_data.recv_new_msg)
	{
		odom_data.recv_new_msg = false;
		return true;
	}

	return false;
}

void PX4CtrlFSM::publish_at_w_from_mpc(const Desired_at_w_t &des_at_W, const ros::Time &stamp)
{
	mavros_msgs::AttitudeTarget msg;

	msg.header.stamp = stamp;
	msg.header.frame_id = std::string("FCU");

	msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

	msg.body_rate.x = des_at_W.wx;
	msg.body_rate.y = des_at_W.wy;
	msg.body_rate.z = des_at_W.wz;

	msg.thrust = des_at_W.ft;

	ctrl_FCU_pub.publish(msg);
}

void PX4CtrlFSM::publish_acc_closeloop_ctrl(const Desired_at_w_t &u, const ros::Time &stamp)
{
	mavros_msgs::VehicleThrustAccSetpoint msg;

	msg.rates_sp[0] = u.wx;
	msg.rates_sp[1] = u.wy;//??   -u.wy
	msg.rates_sp[2] = u.wz;

	msg.thrust_acc_sp = u.ft;//期望的加速度而不是推力
	msg.model_ff= controller.get_thrust_0_1_from_ref_ft(u.ft*param.mass);//get_thrust_0_1_from_ref_ft传进去的是力
	// std::cout<<" u.ft"<< u.ft<<" u.wx"<< u.wx<<" u.wy"<< u.wy<<" u.wz"<< u.wz<<" msg.model_ff "<<msg.model_ff<<std::endl;
	at_w_setpt_pub.publish(msg);
}
void PX4CtrlFSM::publish_bodyrate_ctrl(const Controller_Output_t &u, const ros::Time &stamp)
{
	mavros_msgs::AttitudeTarget msg;

	msg.header.stamp = stamp;
	msg.header.frame_id = std::string("FCU");

	msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

	msg.body_rate.x = u.bodyrates.x();
	msg.body_rate.y = u.bodyrates.y();
	msg.body_rate.z = u.bodyrates.z();

	msg.thrust = u.thrust;

	ctrl_FCU_pub.publish(msg);
}

void PX4CtrlFSM::publish_attitude_ctrl(const Controller_Output_t &u, const ros::Time &stamp)
{
	mavros_msgs::AttitudeTarget msg;

	msg.header.stamp = stamp;
	msg.header.frame_id = std::string("FCU");

	msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
					mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
					mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

	msg.orientation.x = u.q.x();
	msg.orientation.y = u.q.y();
	msg.orientation.z = u.q.z();
	msg.orientation.w = u.q.w();
	// cout << u.q.w() << ',' << u.q.x()<< ',' << u.q.y()<< ',' << u.q.z() << endl;
	msg.thrust = u.thrust;

	ctrl_FCU_pub.publish(msg);
}

void PX4CtrlFSM::publish_trigger(const nav_msgs::Odometry &odom_msg)
{
	geometry_msgs::PoseStamped msg;
	msg.header.frame_id = "world";
	msg.pose = odom_msg.pose.pose;

	traj_start_trigger_pub.publish(msg);
}
void PX4CtrlFSM::publish_offboard_mode()
{
	mavros_msgs::OffboardControlMode msg;
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = true;
	// msg.timestamp = 
	offboard_heartbeat_pub.publish(msg);
}

bool PX4CtrlFSM::apply_rc_stick_abort_offboard_if_needed(const ros::Time &now_time)
{
	const bool rc_ok = rc_is_received(now_time);
	const double thr_norm =
		std::max(0.0, std::min(100.0, param.rc_stick_abort_offboard_percent)) / 100.0;

	if ((int)rc_data.msg.channels.size() >= 4)
	{
		double x[4];
		double max_abs = 0.0;
		for (int i = 0; i < 4; ++i)
		{
			x[i] = ((double)rc_data.msg.channels[i] - 1500.0) / 500.0;
			max_abs = std::max(max_abs, std::fabs(x[i]));
		}
		const bool would_abort = param.rc_stick_abort_offboard_enable && rc_ok && !rc_stick_abort_offboard_latched &&
								 (max_abs > thr_norm);
		ROS_INFO_THROTTLE(
			1.0,
			"[rc_stick_abort dbg] enable=%d latched=%d rc_ok=%d thr_gate=%.4f (param %.1f%%) "
			"ch=[%u,%u,%u,%u] pct_signed=[%.2f%%,%.2f%%,%.2f%%,%.2f%%] max|pct|=%.2f%% %s",
			(int)param.rc_stick_abort_offboard_enable,
			(int)rc_stick_abort_offboard_latched,
			(int)rc_ok,
			thr_norm,
			param.rc_stick_abort_offboard_percent,
			(unsigned)rc_data.msg.channels[0],
			(unsigned)rc_data.msg.channels[1],
			(unsigned)rc_data.msg.channels[2],
			(unsigned)rc_data.msg.channels[3],
			x[0] * 100.0,
			x[1] * 100.0,
			x[2] * 100.0,
			x[3] * 100.0,
			max_abs * 100.0,
			would_abort ? "->TRIGGER" : "");
	}

	// 降落阶段：不因四通道偏移而锁存、切 AUTO_HOVER 或请求 ALTCTL，避免打断 get_takeoff_land_des / land.sh
	if (state == AUTO_LAND)
		return false;

	if (rc_stick_abort_offboard_latched)
		return true;
	if (!param.rc_stick_abort_offboard_enable)
		return false;
	if (!rc_ok)
		return false;
	if ((int)rc_data.msg.channels.size() < 4)
		return false;

	const double thr = thr_norm;
	for (int i = 0; i < 4; ++i)
	{
		const double x = ((double)rc_data.msg.channels[i] - 1500.0) / 500.0;
		if (std::fabs(x) > thr)
		{
			ROS_WARN("[px4ctrl] RC stick abort: TRIGGER axis=%d pwm=%u |norm|=%.4f (thr=%.4f) ch=[%u,%u,%u,%u]",
					 i,
					 (unsigned)rc_data.msg.channels[i],
					 std::fabs(x),
					 thr,
					 (unsigned)rc_data.msg.channels[0],
					 (unsigned)rc_data.msg.channels[1],
					 (unsigned)rc_data.msg.channels[2],
					 (unsigned)rc_data.msg.channels[3]);
			rc_stick_abort_offboard_latched = true;
			if (param.forbid_cmdctrl_reentry_after_loss && state == CMD_CTRL)
				cmdctrl_reentry_forbidden_latched = true;

			if (state == CMD_CTRL || state == AUTO_TAKEOFF)
			{
				const State_t prev = state;
				state = AUTO_HOVER;
				if (prev == AUTO_TAKEOFF)
					set_hov_with_odom();
			}

			request_altctl_mode(now_time, "[px4ctrl] RC stick abort offboard -> request ALTCTL");
			ROS_WARN_THROTTLE(1.0,
							  "[px4ctrl] RC stick abort: latched, skip offboard setpoints until node restart.");
			return true;
		}
	}
	return false;
}

bool PX4CtrlFSM::request_altctl_mode(const ros::Time &now_time, const char *reason)
{
	static constexpr double ALTCTL_RETRY_INTERVAL_SEC = 0.1;

	if (state_data.current_state.mode == "ALTCTL")
	{
		return true;
	}

	if (!last_altctl_request_time.isZero() &&
		(now_time - last_altctl_request_time).toSec() < ALTCTL_RETRY_INTERVAL_SEC)
	{
		return false;
	}

	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "ALTCTL";
	last_altctl_request_time = now_time;

	if (!(set_FCU_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent))
	{
		ROS_ERROR_THROTTLE(1.0, "Exit ALTCTL rejected by PX4!");
		return false;
	}

	ROS_INFO_THROTTLE(1.0, "%s", reason);
	return true;
}


//true就是offboard  
//false就是按遥杆决定manual还是pos  如果是2  offboard 则默认变成posctl  
//这是为了让mpc ctrl(offboard)丢失之后，遥杆还在2offboard  但也可以进入1posctl
bool PX4CtrlFSM::toggle_offboard_mode(bool on_off)
{
	mavros_msgs::SetMode offb_set_mode;
    // std::cout << "[ms=" << (ros::Time::now().toNSec() / 1000000ULL) << "] toggle_offboard_mode " << on_off
    //           << " rc_data.positon_manual_offboard_mode " << rc_data.positon_manual_offboard_mode << std::endl;
	if (on_off)
	{
		// state_data.state_before_offboard = state_data.current_state;
		// if (state_data.state_before_offboard.mode == "OFFBOARD") // Not allowed
		// 	state_data.state_before_offboard.mode = "MANUAL";

		offb_set_mode.request.custom_mode = "OFFBOARD";
		if (!(set_FCU_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent))
		{
			ROS_ERROR_THROTTLE(1.0, "Enter OFFBOARD rejected by PX4!");
			return false;
		}
	}
	else
	{
		offb_set_mode.request.custom_mode = "ALTCTL";//state_data.state_before_offboard.mode;
		// std::cout<<"state_data.state_before_offboard.mode  mostly maybe maunal"<<state_data.state_before_offboard.mode<<std::endl;
		if(rc_data.positon_manual_offboard_mode==1){
			offb_set_mode.request.custom_mode="ALTCTL";
		}
		else if(rc_data.positon_manual_offboard_mode==0){
			offb_set_mode.request.custom_mode="MANUAL";
		}	
		if (!(set_FCU_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent))
		{
			ROS_ERROR_THROTTLE(1.0, "Exit OFFBOARD rejected by PX4!");
			return false;
		}
	}

	return true;

	// if (param.print_dbg)
	// 	printf("offb_set_mode mode_sent=%d(uint8_t)\n", offb_set_mode.response.mode_sent);
}

bool PX4CtrlFSM::toggle_arm_disarm(bool arm)
{
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = arm;
	if (!(arming_client_srv.call(arm_cmd) && arm_cmd.response.success))
	{
		if (arm)
			ROS_ERROR_THROTTLE(1.0, "ARM rejected by PX4! Kill-switch activated?");
		else
			ROS_ERROR_THROTTLE(1.0, "DISARM rejected by PX4!");

		return false;
	}

	return true;
}

void PX4CtrlFSM::reboot_FCU()
{
	// https://mavlink.io/en/messages/common.html, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(#246)
	mavros_msgs::CommandLong reboot_srv;
	reboot_srv.request.broadcast = false;
	reboot_srv.request.command = 246; // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
	reboot_srv.request.param1 = 1;	  // Reboot autopilot
	reboot_srv.request.param2 = 0;	  // Do nothing for onboard computer
	reboot_srv.request.confirmation = true;

	reboot_FCU_srv.call(reboot_srv);

	ROS_INFO_THROTTLE(1.0, "Reboot FCU");

	// if (param.print_dbg)
	// 	printf("reboot result=%d(uint8_t), success=%d(uint8_t)\n", reboot_srv.response.result, reboot_srv.response.success);
}
