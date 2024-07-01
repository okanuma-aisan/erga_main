//ros2 header
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>

//joy board msg
#include <wada_vmc_msgs/msg/can100_20210103.hpp>
#include <wada_vmc_msgs/msg/steer_cmd.hpp>

//autoware msgs
#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>//ウィンカー

//joy board srv
#include <wada_vmc_msgs/srv/blinker.hpp>//ウィンカー
#include <wada_vmc_msgs/srv/actual_input.hpp>//車両操作(can値)

//その他header
#include <wada_vmc/joyboard_receive_info20210103.hpp>
#include <wada_vmc/joyboard_send_info20210103.hpp>
#include <wada_vmc/car_info20210103.hpp>

namespace wada_vmc
{
	class VmcCalculator20210103 : public rclcpp::Node
	{
	private://定数
		constexpr static int TIMER_MESSAGE100_HZ = 100;//timer_message100_タイマーの周期
		constexpr static double TIMER_BLINKER_NEUTRAL_SEC = 0.2;//ウィンカー通達指令時間

	private://joyボード送信用publisher
		rclcpp::Publisher<wada_vmc_msgs::msg::Can100_20210103>::SharedPtr pub_can100_;//joyボード受信データ0x100番のpublisher

	private://joyボード受信用subscriber
		rclcpp::Subscription<wada_vmc_msgs::msg::Can501_20210103>::SharedPtr sub_can501_;//joyボード受信データ0x501番のsubscriber
		rclcpp::Subscription<wada_vmc_msgs::msg::Can502_20210103>::SharedPtr sub_can502_;//joyボード受信データ0x502番のsubscriber
		rclcpp::Subscription<wada_vmc_msgs::msg::Can503_20210103>::SharedPtr sub_can503_;//joyボード受信データ0x503番のsubscriber

	private://autowareからのsubscriber
		rclcpp::Subscription<wada_vmc_msgs::msg::SteerCmd>::SharedPtr sub_steer_cmd_; // ステアコマンド値
		rclcpp::Subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>::SharedPtr sub_actuation_command_;//autowareからの加速度MAPノードから送られるjoyボードコマンド値
		rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr sub_blinker_command_;//autowareからの操舵指令

	private://joyボード制御用サービス
		rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_steer_auto_;//ステアの自動切り替え
		rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_pedal_auto_;//ペダルの自動切り替え
		rclcpp::Service<wada_vmc_msgs::srv::Blinker>::SharedPtr srv_blinker_;//ウィンカーの切り替え
		rclcpp::Service<wada_vmc_msgs::srv::ActualInput>::SharedPtr srv_steer_input_;//ステアの手動入力操作用(joyボードの値をそのまま入力)
		rclcpp::Service<wada_vmc_msgs::srv::ActualInput>::SharedPtr srv_pedal_input_;//ペダルの手動入力操作用(joyボードの値をそのまま入力)
		rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_autoware_input_mode_;//車両操作を手動入力操作からAUTOWARE操作にする
		rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_blinker_input_mode_;//ウィンカー操作を手動入力操作からAUTOWARE操作にする

	private://ros2 timer
		rclcpp::TimerBase::SharedPtr timer_;//通常処理用タイマー
		rclcpp::TimerBase::SharedPtr timer_blinker_neutral_;//ウィンカー指令をニュートラルに戻すタイマー

	private://各種情報
		JoyBoardReceiveInfo joy_receive_info_;//joyボード受信情報
		JoyBoardSendInfo joy_send_info_;//joyボード送信情報
		CarInfo car_info_;//車両固有情報
		rclcpp::Duration life_cycle_;//データの生存時間

	private://joyボード受信用callback
		//joyボード受信データ0x501のcallback
		//[in] message501:ID501番の情報
		void callbackCan501(const wada_vmc_msgs::msg::Can501_20210103::SharedPtr message501)
		{
			joy_receive_info_.add501(*message501, this->now());
		}

		//joyボード受信データ0x502のcallback
		//[in] message502:ID502番の情報
		void callbackCan502(const wada_vmc_msgs::msg::Can502_20210103::SharedPtr message502)
		{
			joy_receive_info_.add502(*message502, this->now());
		}

		//joyボード受信データ0x503のcallback
		//[in] message503:ID503番の情報
		void callbackCan503(const wada_vmc_msgs::msg::Can503_20210103::SharedPtr message503)
		{
			joy_receive_info_.add503(*message503, this->now());
		}

	private://autowareからのcallback
		// ステアコマンド値のcallback
		//[in] cmd:ステアコマンド値
		void callbackSteerCmd(const wada_vmc_msgs::msg::SteerCmd::SharedPtr cmd)
		{
			joy_send_info_.addSteerCommand(cmd->steer_cmd, this->now());
		}

		//AUTOWAREからの加速度MAPから作成したペダルコマンド値
		//[in] cmd:ペダルコマンド値
		void callbackActuationCmd(const tier4_vehicle_msgs::msg::ActuationCommandStamped::SharedPtr cmd)
		{
			double pedal_cmd = (cmd->actuation.accel_cmd == 0.0) ? cmd->actuation.brake_cmd+0.5 : cmd->actuation.accel_cmd-0.5;
			joy_send_info_.addPedalCommand(static_cast<int16_t>(pedal_cmd), this->now());
		}

		//AUTOWAREからpublishされるウィンカー操作コマンドのコールバック
		//[in] blinker:ウィンカー情報
		void callbackBlinkerCommand(const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::SharedPtr blinker)
		{
			joy_send_info_.addBlinkerCommand(blinker->command, this->now());
		}

	private://joyボード制御用サービス
		//ステアの自動切り替え
		//[in] req:ステア自動切り替え指令
		//[out] res:変更内容をそのまま返している
		void serviceSteerAuto(const std_srvs::srv::SetBool::Request::SharedPtr req, const std_srvs::srv::SetBool::Response::SharedPtr res)
		{
			joy_send_info_.setSteerAuto(req->data);
			res->success = joy_send_info_.getSteerAuto();
			res->message = "";
		}

		//ペダルの自動切り替え
		//[in] req:ペダル自動切り替え指令
		//[out] res:変更内容をそのまま返している
		void servicePedalAuto(const std_srvs::srv::SetBool::Request::SharedPtr req, const std_srvs::srv::SetBool::Response::SharedPtr res)
		{
			joy_send_info_.setPedalAuto(req->data);
			res->success = joy_send_info_.getPedalAuto();
			res->message = "";
		}

		//ウィンカーの切り替え
		//[in] req:ウィンカー切り替え指令 wada_vmc_msgs/srv/Blinker.srvを参照
		//[out] res:現在はtrueのみを返す
		void serviceBlinker(const wada_vmc_msgs::srv::Blinker::Request::SharedPtr req, const wada_vmc_msgs::srv::Blinker::Response::SharedPtr res)
		{
			joy_send_info_.setOperationBlnker(false);
			joy_send_info_.addBlinkerCommand(req->blinker, this->now());
			res->success = true;
			timer_blinker_neutral_->reset();
		}

		//ステアの手動入力操作用(joyボードの値をそのまま入力)
		//[in] req:joyボードに入力するステア値
		//[out] res:現在はtrueのみを返す
		void serviceSteerInput(const wada_vmc_msgs::srv::ActualInput::Request::SharedPtr req, const wada_vmc_msgs::srv::ActualInput::Response::SharedPtr res)
		{
			joy_send_info_.setOperationAutoware(false);
			joy_send_info_.clearSteerCommand();
			int16_t st = std::min(std::max(req->input, car_info_.steer_input_right_min_), car_info_.steer_input_left_max_);
			joy_send_info_.addSteerCommand(st, this->now());
			res->success = true;
		}

		//ペダルの手動入力操作用(joyボードの値をそのまま入力)
		//[in] req:joyボードに入力するペダル値
		//[out] res:現在はtrueのみを返す
		void servicePedalInput(const wada_vmc_msgs::srv::ActualInput::Request::SharedPtr req, const wada_vmc_msgs::srv::ActualInput::Response::SharedPtr res)
		{
			joy_send_info_.setOperationAutoware(false);
			joy_send_info_.clearPedalCommand();
			int16_t pd = std::min(std::max(req->input, car_info_.pedal_input_brake_min_), car_info_.pedal_input_accel_max_);
			joy_send_info_.addPedalCommand(pd, this->now());
			res->success = true;
		}

		//車両操作を手動入力操作かたAUTOWARE操作にする
		//[in] req:運用なし
		//[out] res:運用なし
		void serviceAutowareInputMode(const std_srvs::srv::Empty::Request::SharedPtr, const std_srvs::srv::Empty::Response::SharedPtr)
		{
			joy_send_info_.setOperationAutoware(true);
		}

		//ウィンカー操作を手動入力操作かたAUTOWARE操作にする
		//[in] req:運用なし
		//[out] res:運用なし
		void serviceBlinkerInputMode(const std_srvs::srv::Empty::Request::SharedPtr, const std_srvs::srv::Empty::Response::SharedPtr)
		{
			joy_send_info_.setOperationBlnker(true);
		}

	private://ros2 timer
		//初期ロック解除に回る通常処理タイマー
		void callbackTimer()
		{
			publishID100();
		}

		//ウィンカー指令を解除するタイマー
		void callbackTimerBlinkerNeutral()
		{
			joy_send_info_.addBlinkerCommand(autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::NO_COMMAND, this->now());
			timer_blinker_neutral_->cancel();
		}

	private://その他の関数
		//message ID100のデータをros2トピックとして送信
		void publishID100()
		{
			rclcpp::Time ros_nowtime = this->now();
			if(joy_send_info_.getOperationAutoware() == true)//AUTOWAREでの操作になっている場合(サービスからの入力ではない場合)は古い情報を削除
			{
				joy_send_info_.eraseBlinkerCommand(ros_nowtime);
				joy_send_info_.eraseSteerCommand(ros_nowtime);
				joy_send_info_.erasePedalCommand(ros_nowtime);
				joy_send_info_.eraseControlCmd(ros_nowtime);
			}

			//if(joy_receive_info_.get502Size() != 0 && joy_receive_info_.get503Size() != 0)
			{
				const wada_vmc_msgs::msg::Can502_20210103 can502 = joy_receive_info_.get502(0);
				const wada_vmc_msgs::msg::Can503_20210103 can503 = joy_receive_info_.get503(0);

				const int16_t steer_cmd = joy_send_info_.getSteerCommand(0);
				RCLCPP_INFO(this->get_logger(), "steer_cmd,%d", steer_cmd);
				const int16_t pedal_cmd = joy_send_info_.getPedalCommand(0);
				const uint8_t blinker_cmd = joy_send_info_.getBlinkerTrigger();

				//joyボードに送信するmessage ID100番のpublishクラスを準備
				wada_vmc_msgs::msg::Can100_20210103::UniquePtr can100 = std::make_unique<wada_vmc_msgs::msg::Can100_20210103>();
				can100->stamp = ros_nowtime;

				//ステアとペダルの自動設定
				can100->steer_control_mode = (joy_send_info_.getSteerAuto() == true) ? wada_vmc_msgs::msg::Can100_20210103::STEER_AUTO : wada_vmc_msgs::msg::Can100_20210103::STEER_V0;
				can100->pedal_control_mode = (joy_send_info_.getPedalAuto() == true) ? wada_vmc_msgs::msg::Can100_20210103::PEDAL_AUTO : wada_vmc_msgs::msg::Can100_20210103::PEDAL_V0;

				//ステア操舵指令
				if(can502.clutch == false) can100->steer_command = 0;
				else can100->steer_command = (steer_cmd == JoyBoardSendInfo::STEER_COMMAND_INVALID) ? 0 : steer_cmd;
				can100->steer_command = std::min(std::max(can100->steer_command, car_info_.steer_input_right_min_), car_info_.steer_input_left_max_);

				//ペダル操舵指令
				if(can503.clutch == false) can100->pedal_command = 0;
				else can100->pedal_command = (pedal_cmd == JoyBoardSendInfo::PEDAL_COMMAND_INVALID) ? 0 : pedal_cmd;
				can100->pedal_command = std::min(std::max(can100->pedal_command, car_info_.pedal_input_brake_min_), car_info_.pedal_input_accel_max_);

				//ウィンカー
				switch (blinker_cmd)
				{
				case autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_LEFT:
					can100->blinker_left = true;
					break;
				case autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_RIGHT:
					can100->blinker_right = true;
					break;
				case 10:
					can100->hazard_lamp = true;
					break;
				case autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::DISABLE:
					can100->blinker_hazard_cancel = true;
					break;
				}

				//フェイルセーフ
				if(joy_send_info_.getOperationAutoware())
				{
					can100->fail_safe_lighting = car_info_.fail_safe_lighting_;
					can100->steer_fail_safe = car_info_.steer_fail_safe_;
					can100->pedal_fail_safe = car_info_.pedal_fail_safe_;
				}
				else
				{
					can100->fail_safe_lighting = false;
					can100->steer_fail_safe = false;
					can100->pedal_fail_safe = false;
				}

				pub_can100_->publish(std::move(can100));
			}
		}

	public:
		VmcCalculator20210103(const rclcpp::NodeOptions &node_option)
			: rclcpp::Node("wada_vmc_calculation", node_option)
			, joy_receive_info_(this->now())
			, joy_send_info_(this->now())
			, life_cycle_(1, 0)
		{
			car_info_.steer_input_left_max_ = this->declare_parameter<int16_t>("steer_input_left_max", 15000);
			car_info_.steer_input_right_min_ = this->declare_parameter<int16_t>("steer_input_right_min", -15000);
			car_info_.pedal_input_brake_min_ = this->declare_parameter<int16_t>("pedal_input_brake_min", -300);
			car_info_.pedal_input_accel_max_ = this->declare_parameter<int16_t>("pedal_input_accel_max", 450);
			car_info_.steer_input_voltage_min_ = this->declare_parameter<float>("steer_input_voltage_min", 0.0);
			car_info_.steer_input_voltage_max_ = this->declare_parameter<float>("steer_input_voltage_max", 5.0);
			car_info_.pedal_input_voltage_min_ = this->declare_parameter<float>("pedal_input_voltage_min", 0.0);
			car_info_.pedal_input_voltage_max_ = this->declare_parameter<float>("pedal_input_voltage_max", 5.0);

			//joyボード送信用publisher
			pub_can100_ = this->create_publisher<wada_vmc_msgs::msg::Can100_20210103>("can100", rclcpp::QoS(1));

			//joyボード受信用subscriber
			sub_can501_ = this->create_subscription<wada_vmc_msgs::msg::Can501_20210103>("can501", rclcpp::SensorDataQoS(),
				std::bind(&VmcCalculator20210103::callbackCan501, this, std::placeholders::_1));
			sub_can502_ = this->create_subscription<wada_vmc_msgs::msg::Can502_20210103>("can502", rclcpp::SensorDataQoS(),
				std::bind(&VmcCalculator20210103::callbackCan502, this, std::placeholders::_1));
			sub_can503_ = this->create_subscription<wada_vmc_msgs::msg::Can503_20210103>("can503", rclcpp::SensorDataQoS(),
				std::bind(&VmcCalculator20210103::callbackCan503, this, std::placeholders::_1));

			//autowareからのコールバック
						// autowareからのコールバック
			sub_steer_cmd_ = this->create_subscription<wada_vmc_msgs::msg::SteerCmd>("steer_cmd", rclcpp::QoS(1),
				std::bind(&VmcCalculator20210103::callbackSteerCmd, this, std::placeholders::_1));
			sub_actuation_command_ = this->create_subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>(
				"/control/command/actuation_cmd", rclcpp::QoS(1),
				std::bind(&VmcCalculator20210103::callbackActuationCmd, this, std::placeholders::_1));
			sub_blinker_command_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>(
				"/control/command/turn_indicators_cmd", rclcpp::QoS(1),
				std::bind(&VmcCalculator20210103::callbackBlinkerCommand, this, std::placeholders::_1));

			//joyボード制御用サービス
			srv_steer_auto_ = this->create_service<std_srvs::srv::SetBool>("steer_auto_switch",
				std::bind(&VmcCalculator20210103::serviceSteerAuto, this, std::placeholders::_1, std::placeholders::_2),
				rclcpp::ServicesQoS().get_rmw_qos_profile());
			srv_pedal_auto_ = this->create_service<std_srvs::srv::SetBool>("pedal_auto_switch",
				std::bind(&VmcCalculator20210103::servicePedalAuto, this, std::placeholders::_1, std::placeholders::_2),
				rclcpp::ServicesQoS().get_rmw_qos_profile());
			srv_blinker_ = this->create_service<wada_vmc_msgs::srv::Blinker>("blinker_switch",
				std::bind(&VmcCalculator20210103::serviceBlinker, this, std::placeholders::_1, std::placeholders::_2),
				rclcpp::ServicesQoS().get_rmw_qos_profile());
			srv_steer_input_ = this->create_service<wada_vmc_msgs::srv::ActualInput>("steer_input",
				std::bind(&VmcCalculator20210103::serviceSteerInput, this, std::placeholders::_1, std::placeholders::_2),
				rclcpp::ServicesQoS().get_rmw_qos_profile());
			srv_pedal_input_ = this->create_service<wada_vmc_msgs::srv::ActualInput>("pedal_input",
				std::bind(&VmcCalculator20210103::servicePedalInput, this, std::placeholders::_1, std::placeholders::_2),
				rclcpp::ServicesQoS().get_rmw_qos_profile());
			srv_autoware_input_mode_ = this->create_service<std_srvs::srv::Empty>("autoware_input_mode",
				std::bind(&VmcCalculator20210103::serviceAutowareInputMode, this, std::placeholders::_1, std::placeholders::_2),
				rclcpp::ServicesQoS().get_rmw_qos_profile());
			srv_blinker_input_mode_ = this->create_service<std_srvs::srv::Empty>("blinker_input_mode",
				std::bind(&VmcCalculator20210103::serviceBlinkerInputMode, this, std::placeholders::_1, std::placeholders::_2),
				rclcpp::ServicesQoS().get_rmw_qos_profile());

			//ros2 timer
			timer_ = rclcpp::create_timer(this, this->get_clock(), rclcpp::Rate(TIMER_MESSAGE100_HZ).period(),
				std::bind(&VmcCalculator20210103::callbackTimer, this));
			timer_blinker_neutral_ = rclcpp::create_timer(this, this->get_clock(), rclcpp::Rate(TIMER_BLINKER_NEUTRAL_SEC).period(),
				std::bind(&VmcCalculator20210103::callbackTimerBlinkerNeutral, this));
			timer_blinker_neutral_->cancel();//ウィンカー指令をニュートラルに戻すタイマーは、ウィンカー指令があるまで停止
		}
	};
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	std::shared_ptr<wada_vmc::VmcCalculator20210103> node = std::make_shared<wada_vmc::VmcCalculator20210103>(node_options);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}