#ifndef CONTEC_DIO_RECEIVE
#define CONTEC_DIO_RECEIVE

//#include <rclcpp/rclcpp.hpp>
//#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>

#include <rclcpp/rclcpp.hpp>

namespace contec
{
	class ContecDioStatus
	{
	public:
		static constexpr uint8_t BLINKER_NO_COMMAND        		= 255;//ウィンカーダミーデータ用コマンドナンバー

		// DO1　command1
		static constexpr uint8_t DO1_BLINKER_RIGHT               = 0b1000'0000; //ウインカー右　7bit目が1
		static constexpr uint8_t DO1_BLINKER_LEFT                = 0b0100'0000; //ウインカー左　6bit目が1
		static constexpr uint8_t DO1_BLINKER_HAZARD              = 0b1100'0000;//ハザード6,7bit目が1
		static constexpr uint8_t DO1_BLINKER_INTERVENTION        = DO1_BLINKER_HAZARD; //ウィンカー介入判定（右がついているとき、左がつくのは、介入と判断）
		static constexpr uint8_t DO1_BLINKER_DISABLE             = 0b0000'0000;//6,7bit目を0にするとウィンカー消去 
		static constexpr uint8_t DO1_BLINKER_ACQUISITION         = 0b1100'0000;//CONTECデバイスから取得したバイトデータから、ウィンカー情報のみを[&]で取得する際のビット指定値
		static constexpr uint8_t DO1_HANDLE_CLUTCH_BAN           = 0b0001'0000;//4bit目が1 ハンドルクラッチ断OUTPUT (1の間だけ切れている)
		static constexpr uint8_t DO1_PEDAL_CLUTCH_BAN            = 0b0000'1000;//3bit目が1 ペダルクラッチ断OUTPUT  (1の間だけ切れている)
		static constexpr uint8_t DO1_HANDLE_SET                  = 0b0000'0100;//ハンドルセットOUTPUT 2bit目が1　contec側でハンドルセットができる工夫。これはレベル４想定。
		static constexpr uint8_t DO1_HANDLE_LED                  = 0b0000'0010;//ハンドル自動可否ＬＥＤ　ハンドル 1bit目が1で点灯（自動不可)
		static constexpr uint8_t DO1_PEDAL_LED                   = 0b0000'0001;//ペダル自動可否ＬＥＤ　ペダル　1bit目が1で点灯（自動不可)
		//DO0 command0
		static constexpr uint8_t DO0_HANDLE_FAILSAFE             = 0b1000'0000; //ハンドルフェイルセーフランプOUTPUT 7bit目が1で点灯（切断保持の間だけ点灯）
		static constexpr uint8_t DO0_PEDAL_FAILSAFE              = 0b0100'0000; //ペダルフェイルセーフランプOUTPUT　 6bit目が1で点灯（切断保持の間だけ点灯）
		
		//DI1 in_port1data
		static constexpr uint8_t DI1_BLINKER_RIGHT               = 0b1000'0000; //ウインカー右　7bit目が1
		static constexpr uint8_t DI1_BLINKER_LEFT                = 0b0100'0000; //ウインカー左　6bit目が1
		static constexpr uint8_t DI1_BLINKER_HAZARD              = 0b1100'0000; //ハザード
		static constexpr uint8_t DI1_BLINKER_DISABLE             = 0b0000'0000; //ウィインカーなし
		static constexpr uint8_t DI1_HANDLE_MAIN                 = 0b0010'0000;//ハンドルメイン(5bit目)INPUT(ハンドルメインのトグルスイッチが上がると１)
		static constexpr uint8_t DI1_PEDAL_MAIN                  = 0b0001'0000;//ペダルメイン(4bit目)INPUT(ペダルメインのトグルスイッチが上がると１)
		static constexpr uint8_t DI1_HANDLE_TAKEOVER             = 0b0000'1000;//ハンドルテイクオーバーINPUT
		static constexpr uint8_t DI1_BRAKE_TAKEOVER              = 0b0000'0100;//ブレーキテイクオーバーINPUT
		static constexpr uint8_t DI1_HANDLE_SET                  = 0b0000'0010;//ハンドルセットINPUT(運転士のハンドルセットボタンに連動)
		static constexpr uint8_t DI1_ACCEL_TAKEOVER              = 0b0000'0001;//アクセルテイクオーバーINPUT
		//DI0 in_port0data
		static constexpr uint8_t DI0_HANDLE_DISCONNECTION        = 0b1000'0000;//ハンドル制御ボードとPCの間のCAN断線　INPUT
		static constexpr uint8_t DI0_PEDAL_DISCONNECTION         = 0b0100'0000;//ペダル制御ボードとPCの間のCAN断線　INPUT
		static constexpr uint8_t DI0_PEDAL_SET                   = 0b0010'0000; //ペダルセットINPUT
		static constexpr uint8_t DI0_DRIVER1                     = 0b0001'0000; //ドライバー入力1INPUT
		static constexpr uint8_t DI0_DRIVER2                     = 0b0000'1000; //ドライバー入力2INPUT
		static constexpr uint8_t DI0_ALIGHTING                   = 0b0000'0100; //降車ランプ点灯INPUT
		static constexpr uint8_t DI0_PEDAL_CLUTCH_LATCH          = 0b0000'0010; //ペダルクラッチ保持非保持INPUT（運転士さんの設定）
		//onの時に、ブレーキテイクオーバーの保持をしない。→　ID100の1byte 5bit目を1にして、ブレーキテイクオーバーの保持をしないコードを追加する。	
		static constexpr uint8_t DI0_BLINKER_CANCEL              = 0b0000'0001; //ウィンカーキャンセルINPUT

	private://ウィンカー用メンバ変数
		std::vector<rclcpp::Time> blinker_create_time_;//ウィンカーデータを作成した時間の履歴
		std::vector<uint8_t> blinker_que_;//左ウィンカー点灯状態の履歴

	private://降車ボタン用メンバ変数
		std::vector<rclcpp::Time> alighting_create_time_;//降車ボタンデーターを作成した時間の履歴
		std::vector<uint8_t> alighting_que_;//降車ボタンデーター状態の履歴

	private://ハンドルメイン用メンバ変数
		std::vector<rclcpp::Time> handle_main_time_;//ハンドルメインを作成した時間の履歴
		std::vector<bool> handle_main_que_;//ハンドルメインの履歴

	private://ペダルメイン用メンバ変数
		std::vector<rclcpp::Time> pedal_main_time_;//ペダルメインを作成した時間の履歴
		std::vector<bool> pedal_main_que_;//ペダルメインの履歴

	private://その他のメンバ変数
		rclcpp::Duration life_cycle_;//blinker_que_内データの生存時間

	private://メンバ関数
		//blinker_que_のサイズが0の場合に実行し、ダミーデータを挿入
		//[in] nowtime:現在の時間
		void insertEmptyData(const rclcpp::Time &nowtime)
		{
			blinker_create_time_.push_back(nowtime);
			blinker_que_.push_back(BLINKER_NO_COMMAND);
		}

	public://初期化
		//[in] nowtime:現在の時間
		//[in] life_cycle:ウィンカー情報を登録してから、情報を消去する時間
		ContecDioStatus(const rclcpp::Time &nowtime, const rclcpp::Duration &life_cycle)
			: life_cycle_(life_cycle)
		{
			insertEmptyData(nowtime);
		}

	public://CONTECのデータ全体に関する関数
		//データの生存時間を取得
		//返り値:データの生存時間
		rclcpp::Duration getLifeCycle() const
		{
			return life_cycle_;
		}

		//生存時間を過ぎたデータを削除
		//[in] nowtime:現在の時間
		void pops(const rclcpp::Time &nowtime)
		{
			size_t i;
			for(i=0; i<blinker_que_.size(); i++)
			{
				if(nowtime - blinker_create_time_[i] > life_cycle_)
				{
					blinker_que_.erase(blinker_que_.begin() + i, blinker_que_.end());
					blinker_create_time_.erase(blinker_create_time_.begin() + i, blinker_create_time_.end());
					break;
				}
			}

			if(blinker_que_.size() == 0) insertEmptyData(nowtime);
		}

	public://ウィンカーの追加・取得・判定の関数
		//ウィンカー情報の取得
		//[in] index:取得したいウィンカー情報キューのインデックス
		//返り値:指定したindexのウィンカー情報
		uint8_t getBlinker(const size_t index) const
		{
			return blinker_que_[index];
		}

		//新しいウィンカ情報を追加
		//[in] blinker:新しいウィンカー情報
		//[in] nowtime:現在の時間
		void addBlinker(const uint8_t blinker, const rclcpp::Time &nowtime)
		{
			blinker_que_.insert(blinker_que_.begin(), blinker);
			blinker_create_time_.insert(blinker_create_time_.begin(), nowtime);
		}

		//ウィンカー情報キューのサイズ取得
		//返り値:ウィンカー情報キューのサイズ
		size_t blinkerQueSize() const 
		{
			return blinker_que_.size();
		}

		//ウィンカー履歴の全体の時間を取得
		//返り値:ウィンカー履歴の全体の時間
		rclcpp::Duration blinkerAllTime() const
		{
			return blinker_create_time_[0] - blinker_create_time_[blinker_create_time_.size()-1];
		}

	public://降車ボタン情報の追加・取得・判定の関数
		//降車ボタン情報の取得
		//[in] index:取得したい降車ボタン情報キューのインデックス
		//返り値:指定したindexの降車ボタン情報
		uint8_t getAlighting(const size_t index) const
		{
			return alighting_que_[index];
		}

		//新しい降車ボタン情報を追加
		//[in] alighting:新しい降車ボタン情報
		//[in] nowtime:現在の時間
		void addAlighting(const uint8_t alighting, const rclcpp::Time &nowtime)
		{
			alighting_que_.insert(alighting_que_.begin(), alighting);
			alighting_create_time_.insert(alighting_create_time_.begin(), nowtime);
		}

		//降車ボタン情報キューのサイズ取得
		//返り値:降車ボタン情報キューのサイズ
		size_t alightingQueSize() const 
		{
			return alighting_que_.size();
		}

	public://ハンドルメイン情報の追加・取得・判定の関数
		//ハンドルメイン情報の取得
		//[in] index:取得したいハンドルメイン情報キューのインデックス
		//返り値:指定したindexのハンドルメイン情報
		bool getHandleMain(const size_t index) const
		{
			return handle_main_que_[index];
		}

		//新しいハンドルメイン情報を追加
		//[in] alighting:新しいハンドルメイン情報
		//[in] nowtime:現在の時間
		void addHandleMain(const bool handle_main_, const rclcpp::Time &nowtime)
		{
			handle_main_que_.insert(handle_main_que_.begin(), handle_main_);
			handle_main_time_.insert(handle_main_time_.begin(), nowtime);
		}

		//ハンドルメイン情報キューのサイズ取得
		//返り値:ハンドルメイン情報キューのサイズ
		size_t handleMainQueSize() const 
		{
			return handle_main_que_.size();
		}

	public://ペダルメイン情報の追加・取得・判定の関数
		//ペダルメイン情報の取得
		//[in] index:取得したいペダルメイン情報キューのインデックス
		//返り値:指定したindexのペダルメイン情報
		bool getPedalMain(const size_t index) const
		{
			return pedal_main_que_[index];
		}

		//新しいペダルメイン情報を追加
		//[in] alighting:新しいペダルメイン情報
		//[in] nowtime:現在の時間
		void addPedalMain(const bool pedal_main_, const rclcpp::Time &nowtime)
		{
			pedal_main_que_.insert(pedal_main_que_.begin(), pedal_main_);
			pedal_main_time_.insert(pedal_main_time_.begin(), nowtime);
		}

		//ペダルメイン情報キューのサイズ取得
		//返り値:ペダルメイン情報キューのサイズ
		size_t pedalMainQueSize() const
		{
			return pedal_main_que_.size();
		}
	};

	constexpr uint8_t ContecDioStatus::BLINKER_NO_COMMAND;
}
#endif
