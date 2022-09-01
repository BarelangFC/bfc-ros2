#include <unistd.h>
#include <sys/time.h>

#include "rclcpp/rclcpp.hpp"
#include "bfc_msgs/msg/button.hpp"
#include "bfc_msgs/msg/head_movement.hpp"
#include "bfc_msgs/msg/coordination.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

#define PI 3.1415926535897932384626433832795

using namespace std;

class main_strategy : public rclcpp::Node
{
public:
    main_strategy() : Node("main_strategy")
    {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(40),
            std::bind(&main_strategy::barelang, this));

        declareParameters();
        getParameters();

        robotNumber = this->get_parameter("robotNumber").as_int();

        button_ = this->create_subscription<bfc_msgs::msg::Button>(
            "robot_button", 10,
            std::bind(&main_strategy::readButton, this, std::placeholders::_1));
        imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "robot_imu", 10,
            std::bind(&main_strategy::readImu, this, std::placeholders::_1));
        bounding_boxes_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "robot_vision", 10,
            std::bind(&main_strategy::objCoor, this, std::placeholders::_1));
        gameControllerSubscription_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
            "robot_game_controller", 10,
            std::bind(&main_strategy::readGameControllerData, this, std::placeholders::_1));

        if (robotNumber != 1)
        {
            robot1Subscription_ = this->create_subscription<bfc_msgs::msg::Coordination>(
                "/robot_1/coordination", 10, std::bind(&main_strategy::readRobotCoordinationData1, this, std::placeholders::_1));
        }

        if (robotNumber != 2)
        {
            robot2Subscription_ = this->create_subscription<bfc_msgs::msg::Coordination>(
                "/robot_2/coordination", 10, std::bind(&main_strategy::readRobotCoordinationData2, this, std::placeholders::_1));
        }

        if (robotNumber != 3)
        {
            robot3Subscription_ = this->create_subscription<bfc_msgs::msg::Coordination>(
                "/robot_3/coordination", 10, std::bind(&main_strategy::readRobotCoordinationData3, this, std::placeholders::_1));
        }

        if (robotNumber != 4)
        {
            robot4Subscription_ = this->create_subscription<bfc_msgs::msg::Coordination>(
                "/robot_4/coordination", 10, std::bind(&main_strategy::readRobotCoordinationData4, this, std::placeholders::_1));
        }

        if (robotNumber != 5)
        {
            robot5Subscription_ = this->create_subscription<bfc_msgs::msg::Coordination>(
                "/robot_5/coordination", 10, std::bind(&main_strategy::readRobotCoordinationData5, this, std::placeholders::_1));
        }

        robotCoordination_ = this->create_publisher<bfc_msgs::msg::Coordination>(
            "coordination", 10);
        cmd_head_ = this->create_publisher<bfc_msgs::msg::HeadMovement>(
            "robot_head", 10);
        cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "robot_walk", 10);
        cmd_mot_ = this->create_publisher<std_msgs::msg::String>(
            "robot_motion", 10);
    }

    void display()
    {
        cout << "Ball_X = " << Ball_X << ", Ball_Y = " << Ball_Y << endl;
        cout << "Goal_X = " << Goal_X << ", Goal_Y = " << Goal_Y << endl;
        cout << "frame_X = " << frame_X << ", frame_Y = " << frame_Y << endl;
        cout << "Stretegy = " << msg_strategy << endl;
        cout << "Play = " << boolalpha << play << endl;
        cout << "useCoordination = " << boolalpha << useCoordination << endl;
        cout << "useVision = " << boolalpha << useVision << endl;
        cout << "firstStateCondition = " << firstStateCondition << endl;
        cout << "StateCondition = " << stateCondition << endl;
        cout << "StateGameController = " << State << endl;
        cout << "kickOff = " << KickOff << endl;
        cout << "barelang_color = " << barelang_color << endl;
        cout << "signIn = " << boolalpha << signIn << endl;
        cout << "headPan = " << headPan << ", headTilt = " << headTilt << endl;

        cout << "\033[2J\033[1;1H";
    }

    void barelang()
    {
        getSensor();
        sudut();
        cekArah();
        display();

        if (useLocalization)
        {
            if (strategyCM != 4)
            {
                getServPos();
                gridLocalization();
                gridBall();
                gridCoor();
                // bacaKondisi(RobotWalk_X, RobotWalk_Y);
                // mapping(uTorsoActual[0], uTorsoActual[1]);
                robotPos_X = deltaPos_X + initialPos_X;
                robotPos_Y = deltaPos_Y + initialPos_Y;

                RobotPos_X = robotPos_X;
                RobotPos_Y = robotPos_Y;
                // trigonoMetri();
                // get_invers();
                // calc_fk();
            }
        }
        else
        {
            followSearchAktif = true;
        }

        if (useLastDirection == true && lastDirection != 0)
        {
            if ((angle < lastDirection + 15) && (angle > lastDirection - 15))
            {
                useImu = false;
                // robotDirection = true;
            }
            else
            {
                lastDirection = 0;
                useImu = true;
                // robotDirection = true;
            }
        }

        // INITIAL POSISI
        //  if (modePlay == 0) { //INTIAL POSITION NASIONAL
        //  	if(useGameController) {
        //  		if(!play && State == 0 && !usePenaltyStrategy) {
        //  			if ((!pickUp && Remaining == 600 &&  SecondaryState == 0) || (!pickUp && Remaining == 300 &&  SecondaryState == 2)) {

        // 					if (angle > -30 && angle < 30) {	//Initial nilai odometry masuk tengah
        // 						/* if (countInitPos == 0) {
        // 							initialPos_X = -90;
        // 							initialPos_Y = -70;
        // 							initGrid = 21; 	offsetX = 50; offsetY = 50;
        // 							if (robotStatus == 1) {
        // 								countInitPos = 1;
        // 							} else {
        // 								countInitPos = 0;
        // 							}
        // 						} */
        // 						modeImu = 0;

        // 					} else {								//Initial nilai odometry masuk samping
        // 						if ( angle > 30 && countInitPos == 0) {				//Support Kanan
        // 							/* initialPos_X = -358;
        // 							initialPos_Y = 130;
        // 							initGrid = 22; 	offsetX = -50; offsetY = 20;
        // 							if (robotStatus == 1) {
        // 								countInitPos = 1;
        // 							} else {
        // 								countInitPos = 0;
        // 							} */
        // 							modeImu = 1;
        // 						} else if (angle < 30 && countInitPos == 0) {			//Support Kiri
        // 							/* initialPos_X = -358;
        // 							initialPos_Y = -130;
        // 							initGrid = 21; 	offsetX = -50; offsetY = -20;
        // 							if (robotStatus == 1) {
        // 								countInitPos = 1;
        // 							} else {
        // 								countInitPos = 0;
        // 							} */
        // 							modeImu = 1;
        // 						}

        // 					}
        // 			}
        // 		} else if (play && !usePenaltyStrategy) {
        // 			if ((pickUp && Remaining != 600 && SecondaryState == 0) || (pickUp && Remaining != 300 && SecondaryState == 2)) {
        // 				if (angle >= 0 && countInitPos == 0) {
        // 					initialPos_X = -75;
        // 					initialPos_Y = -305;
        // 				} else if (angle < 0 && countInitPos == 0) {
        // 					initialPos_X = -75;
        // 					initialPos_Y = 305;
        // 				}
        // 			}
        // 		} else if (!play && State == 0 && usePenaltyStrategy) {
        // 			initialPos_X = 150;
        // 			initialPos_Y = 0;
        // 		}
        // 	} else {
        // 		if(strategyCM == 1){
        // 			modeImu = 0;
        // 		}else if(strategyCM == 2){
        // 			modeImu = 1;
        // 		}
        // 		// if (!play && !usePenaltyStrategy){
        // 		// 	initialPos_X = -350;
        // 		// 	initialPos_Y = 0;
        // 		// } else if (!play && usePenaltyStrategy) {
        // 		// 	initialPos_X = 150;
        // 		// 	initialPos_Y = 0;
        // 		// }
        // 	}
        // } else if (modePlay == 1) {	//INTIAL POSITION INTERNASIONAL
        // 	if(useGameController) {
        // 		if(!play && State == 0 && !usePenaltyStrategy) {
        // 			if ((!pickUp && Remaining == 600 &&  SecondaryState == 0) || (!pickUp && Remaining == 300 &&  SecondaryState == 2)) {
        // 				if (countInitPos == 0) {
        // 					if (KickOff == barelang_color || KickOff == dropball) { //Attack
        // 						if (strategyCM < 3) {	//masuk depan
        // 							if (angle >= 0) {
        // 								initialPos_X = -130; initialPos_Y = -300;
        // 								initGrid = 21; 	offsetX = 50; offsetY = 50;
        // 							} else if (angle < 0) {
        // 								initialPos_X = -130; initialPos_Y = 300;
        // 								initGrid = 22; offsetX = 50; offsetY = -50;
        // 							}
        // 						} else if (strategyCM == 3) {	//masuk samping depan
        // 							if (angle >= 0) {
        // 								initialPos_X = -300; initialPos_Y = -305;
        // 								initGrid = 20; 	offsetX = 0; offsetY = 0;
        // 							} else if (angle < 0) {
        // 								initialPos_X = -300; initialPos_Y = 305;
        // 								initGrid = 23; offsetX = 0; offsetY = 0;
        // 							}
        // 						}
        // 					} else {	//Defense
        // 						if (strategyCM < 3) {	//masuk depan
        // 							if (angle >= 0) {
        // 								initialPos_X = -130; initialPos_Y = -300;
        // 								initGrid = 21; 	offsetX = -50; offsetY = 50;
        // 							} else if (angle < 0) {
        // 								initialPos_X = -130; initialPos_Y = 300;
        // 								initGrid = 22; offsetX = -50; offsetY = -50;
        // 							}
        // 						} else if (strategyCM == 3) {	//masuk samping belakang
        // 							if (angle >= 0) {	//Kiri
        // 								initialPos_X = -300; initialPos_Y = -305;
        // 								initGrid = 14; 	offsetX = 0; offsetY = 0;
        // 							} else if (angle < 0) {	//Kanan
        // 								initialPos_X = -300; initialPos_Y = 305;
        // 								initGrid = 17; offsetX = 0; offsetY = 0;
        // 							}
        // 						}
        // 					}
        // 					if (robotStatus == 1) {
        // 						countInitPos = 1;
        // 					} else {
        // 						countInitPos = 0;
        // 					}
        // 				}
        // 			}
        // 		} else if (play && !usePenaltyStrategy || switched) {
        // 			if ((pickUp && Remaining != 600 && SecondaryState == 0) || (pickUp && Remaining != 300 && SecondaryState == 2)) {
        // 				if (angle >= 0 && countInitPos == 0) {
        // 					//printf(" masuk setingan pickUp balikKiri\n");
        // 					initialPos_X = -300;
        // 					initialPos_Y = -305;
        // 					masukKiri = true;
        // 					masukKanan = false;
        // 					countInitPos = 1;
        // 				} else if (angle < 0 && countInitPos == 0) {
        // 					//printf(" masuk setingan pickUp balikKanan\n");
        // 					initialPos_X = -300;
        // 					initialPos_Y = 305;
        // 					masukKanan = true;
        // 					masukKiri = false;
        // 					countInitPos = 1;
        // 				}
        // 			}
        // 		} else if (!play && State == 0 && usePenaltyStrategy) {
        // 			initialPos_X = 240;
        // 			initialPos_Y = 0;
        // 		}
        // 	} else {
        // 		if (!play && !usePenaltyStrategy){
        // 			initialPos_X = -350;
        // 			initialPos_Y = 0;
        // 		} else if (!play && usePenaltyStrategy) {
        // 			initialPos_X = 240;
        // 			initialPos_Y = 0;
        // 		}
        // 	}
        // } else {/*nothing*/}

        if (useCoordination)
        {
            refreshComm(); // refresh data communications

            if (zeroState == true || play == false || stateCondition == 1 || stateCondition == 150 || stateCondition == 200)
            {
                countHilang = 0;
                semeh = 232;
                sendRobotCoordinationData(robotNumber, robotStatus, stateCondition, Grid, 0, semeh, GridBall, backIn);
            }
            else if (stateCondition == 7 || stateCondition == 3 || stateCondition == 8 || stateCondition == 4 || stateCondition == 5 || stateCondition == -10 || stateCondition == 10 || stateCondition == 20 || stateCondition == 30)
            {
                countHilang = 0;
                semeh = 1;
                sendRobotCoordinationData(robotNumber, robotStatus, 232, Grid, 1, semeh, GridBall, backIn);
            }
            else if (stateCondition == 2 || stateCondition == 6)
            {
                // langsung kirim didalam case
            }
            else
            { // sisa stateCondition yang tidak dikondisikan
                if (Ball_X == -1 && Ball_Y == -1)
                { // bola hilang
                    if (countHilang > 20)
                    {
                        semeh = 232;
                        sendRobotCoordinationData(robotNumber, robotStatus, stateCondition, Grid, 0, semeh, GridBall, backIn);
                    }
                    else
                    {
                        countHilang++;
                    }
                }
                else
                { // dapat bola
                    countHilang = 0;
                    trackBall();
                    sendRobotCoordinationData(robotNumber, robotStatus, stateCondition, Grid, 1, semeh, GridBall, backIn);
                }
            }
        }

        if (strategyCM == 4)
        { // printf("  check receler\n");
            motion("0");
            resetAllVariable();

            getSensor();

            State = 0;
            stateGameController = 0;
            lastStateGameController = 0;

            play = false;

            backIn = pickUp = false;
            // firstTimes = true;
            manual = true;
            countInitPos = 0;

            robotStatus = 0;

            if (ballLost(20))
            {
                tiltSearchBall(0.0);
            }
            else
            {
                trackBall();
            }
            stateCondition = 150;
        }
        else
        {
            if (state == 0)
            {
                if (backIn)
                {
                    robotStatus = 0;
                }
                else
                {
                    robotStatus = 1;
                }
            }
        }

        if ((Penalty1 == 5 && timNumber1 == team) || (Penalty2 == 5 && timNumber2 == team))
        {
            zeroState = true;
            signIn = false;
        }
        else
        {
            signIn = true;
        }

        if (stateGameController == 3)
        {
            if (robotFall == true)
            {
                resetCase1();
                jatuh = true;
            }
            else if (robotFall == false && jatuh)
            {
                // motion("9");
                jatuh = false;
                stateCondition = 1;
            }
        }

        // Untuk Cek Imu, Jika Selalu 0 maka ke case 200
        //		if (strategy.bearingValue == 0) {
        //			countBearing += 1;
        //			if (countBearing >= 200) {
        //				stateCondition = 200;
        //			}
        //		} else {
        //			countBearing = 0;
        //		}

        ////////////////////////////////////////////////////////////////////////
        ////////////////.............Game Controller.............///////////////
        ////////////////////////////////////////////////////////////////////////
        if (useGameController)
        {
            stateGameController = State;
            //			if (lastStateGameController != stateGameController) {
            //				if (stateGameController > lastStateGameController) { //printf("maju\n");
            //					if (stateChange > 30) { //40
            //						lastStateGameController = stateGameController;
            //					} else { stateChange++; }
            //				} else if (stateGameController < lastStateGameController) { //printf("mundur\n");
            //					if (
            //						(lastStateGameController == 4 && stateGameController == 0) ||
            //						(lastStateGameController == 3 && stateGameController == 1)
            //					   ) {
            //						if (stateChange > 30) { //40
            //							lastStateGameController = stateGameController;
            //						} else { stateChange++; }
            //					}
            //				}
            //			} else { stateChange = 0; }

            /*kickOff = KickOff;
            if (lastKickOff != kickOff) {
                if (kickOffChange > 40) { //30
                    lastKickOff = kickOff;
                } else { kickOffChange++; }
            } else { kickOffChange = 0; }


            secRemaining = Remaining;
            if (lastSecRemaining != secRemaining) {
                if (secRemainingChange > 40) { //30
                    lastSecRemaining = secRemaining;
                } else { secRemainingChange++; }
            } else { secRemainingChange = 0; }*/

            // switch (lastStateGameController) {
            switch (stateGameController)
            {

            case 0: // printf("  Initial\n\n");
                zeroState = false;
                play = false;

                if (strategyCM != 4)
                {
                    motion("0"); // Walk(0.0, 0.0, 0.0);
                    // headMove(0.0, -1.1);
                    // sudut();
                    predictGoal(angle, -1.6);

                    backPosition = false;
                    resetAllVariable();
                    cset = 0;

                    stateCondition = 150;

                    // posRotate = false;
                    // setWaktu();
                }
                break;

            case 1: // printf("  Ready\n\n\n");
                zeroState = false;
                play = false;
                if (strategyCM != 4)
                {
                    // if (backIn) {
                    // 	motion("0");
                    // 	robotStatus = 0;
                    // 	predictGoal(angle, -1.6);
                    // } else {

                    robotPositioning(modeImu);
                    // }
                    // stateCondition = 150;
                    // }else if(strategyCM == 3){
                    // 	robotPositioning(modeImu);
                }
                break;

            case 2: // printf("  \n\n\nSet\n\n\n\n");
                zeroState = false;
                play = false;
                cset = 0;
                second = 0;
                timer = false;
                kurama = 0;
                resetKoordinasiRobotBalik();
                if (strategyCM != 4)
                {
                    motion("0");

                    firstTimes = true;
                    pickUp = false;
                    switched = false;
                    kondisiBola = 0;
                    stateCondition = 0;
                    reset = 0;
                    countDef = 0;
                    cntStabilize0 = 0;
                    if (backIn)
                    {
                        robotStatus = 1;
                        backIn = false;
                    }
                    resetCaseAwal();
                    stateCondition = 150;
                    refreshMoveGrid();

                    if (ballLost(20))
                    {
                        // SearchBall(2);
                        searchBallRectang(-1.5, -1.6, -0.8, 1.6);
                    }
                    else
                    {
                        trackBall();
                    }
                }
                break;

            case 3: // printf("play\n");
                kurama = 0;
                if (strategyCM != 4)
                {
                    if (firstTimes)
                    {
                        searchKe = 0;
                        timer = false;
                        resetCaseAwal();
                        // resetAllVariable();

                        stateCondition = firstStateCondition;
                        firstTimes = false;
                        play = true;
                    }
                }
                break;

            case 4: // printf("finish\n");
                zeroState = false;
                kurama = 0;
                motion("0");
                // motion("7");
                predictGoal(angle, -1.6);

                play = false;
                manual = false;
                backPosition = false;
                resetAllVariable();
                stateCondition = 150;
                break;

            default:
                break;
            }
        }
        else
        {
            if (firstTimes)
            {
                backPosition = false;
                resetAllVariable();
                stateCondition = firstStateCondition;
                firstTimes = false;
            }
            play = true;
            backIn = false;
        }

        ////////////////////////////////////////////////////////////////////////
        ////////////////.............Strategy Button.............///////////////
        ////////////////////////////////////////////////////////////////////////
        // state = killNrunCM;
        if (killNrun == 1)
        { // printf("killall\n")
            if (wait < 20)
            {
                wait++;
            }
            else
            {
                state = 1;
                if (stateGameController == 3 || lastStateGameController == 3)
                {
                    zeroState = true;
                }
            }
        }
        else
        { // printf("state run lua\n");
            wait = 0;
            state = 0;
        }

        // printf("wait = %d, state = %d\n",wait,state);

        if (lastState != state)
        {
            if (state == 1)
            { // printf("killall\n");
                resetAllVariable();
                resetOdometry();
                refreshRecelerLokalisasi();

                stateCondition = 150;
                countInitPos = 0;
                robotStatus = 0;
                motion("0");

                backIn = pickUp = false;
                // Penalise = false;
                // system("killall screen;");
                // runDcm();
                // runLuaProgram();
            }
            else if (state == 0)
            { // printf("state run lua\n");
                // runLuaProgram();
                motion("8");
                motion("0");
                // printf("RUNNNNNNNNNNN\n");
                getParameters();
                zeroState = false; // didnt find ball for coor
                backPosition = false;
                kurama = 0;
                manual = false;
                reset = 0;

                if (stateGameController == 1)
                {
                    robotStatus = 0;
                    backIn = true;
                    switched = true;
                }
                else
                {
                    robotStatus = 1;
                    backIn = false;
                    switched = false;
                }

                State = 3;
                stateGameController = 3;
                lastStateGameController = 3;

                // Penalise = true;

                play = true;

                pickUp = true;
                stateCondition = firstStateCondition;
            }
            lastState = state;
        }

        ///////////////////////////////////////////////////////////////////////
        //////////////.............Role of execution............///////////////
        ///////////////////////////////////////////////////////////////////////
        if (play)
        { // printf("BarelangFC Rock n Roll\n");
            switch (stateCondition)
            {
            case 0: // First Strategy
                if (!backPosition)
                {
                    Strategi = strategyCM;
                }
                else
                {
                    Strategi = 0;
                }

                switch (Strategi)
                {
                case 0: // Serang lurus - Defense depan di tengah
                    if (KickOff == barelang_color || KickOff == dropball)
                    { //&& modeImu == 0) { //Attack
                        modeImu = 0;
                        if (Ball_X != -1 && Ball_Y != -1)
                        {
                            trackBall();
                        }

                        if (pickUp)
                        {
                            if (signIn)
                            {
                                motion("9");
                                setWaktu();
                                stateCondition = 50;
                            }
                            else
                            {
                                motion("0");
                                headMove(0.0, -1.4);
                            }
                        }
                        else
                        {
                            motion("9");
                            if (usePenaltyStrategy)
                            {
                                useFollowSearchGoal = false;
                                useSearchGoal = false;
                            }
                            stateCondition = -10; //-10; //-20;
                        }
                    }
                    else
                    { // Defense
                        modeImu = 2;
                        if (Ball_X != -1 && Ball_Y != -1)
                        {
                            trackBall();
                        }

                        if (pickUp)
                        {
                            if (signIn)
                            {
                                motion("9");
                                setWaktu();
                                stateCondition = 50;
                            }
                            else
                            {
                                motion("0");
                                headMove(0.0, -1.4);
                            }
                        }
                        else
                        {
                            motion("0");
                            setWaktu();
                            stateCondition = 130;
                        }
                    }
                    break;

                case 1: // serang ke kanan - Defense belakang di kiri/kanan
                    if (KickOff == barelang_color || KickOff == dropball)
                    { //&& modeImu == 0) { //Attack
                        modeImu = 0;
                        if (Ball_X != -1 && Ball_Y != -1)
                        {
                            trackBall();
                        }

                        if (pickUp)
                        {
                            if (signIn)
                            {
                                motion("9");
                                setWaktu();
                                stateCondition = 50;
                            }
                            else
                            {
                                motion("0");
                                headMove(0.0, -1.4);
                            }
                        }
                        else
                        {
                            motion("9");
                            if (usePenaltyStrategy)
                            {
                                useFollowSearchGoal = false;
                                useSearchGoal = false;
                            }
                            stateCondition = 10;
                        }
                    }
                    else
                    { // Defense
                        modeImu = 2;
                        if (Ball_X != -1 && Ball_Y != -1)
                        {
                            trackBall();
                        }

                        if (pickUp)
                        {
                            if (signIn)
                            {
                                motion("9");
                                setWaktu();
                                stateCondition = 50;
                            }
                            else
                            {
                                motion("0");
                                headMove(0.0, -1.4);
                            }
                        }
                        else
                        {
                            motion("0");
                            setWaktu();
                            stateCondition = 130;
                        }
                    }
                    break;

                case 2: // serang ke kiri - Defense belakang di kiri/kanan
                    if (KickOff == barelang_color || KickOff == dropball)
                    { //&& modeImu == 0) { //Attack
                        modeImu = 0;
                        if (Ball_X != -1 && Ball_Y != -1)
                        {
                            trackBall();
                        }

                        if (pickUp)
                        {
                            if (signIn)
                            {
                                motion("9");
                                setWaktu();
                                stateCondition = 50;
                            }
                            else
                            {
                                motion("0");
                                headMove(0.0, -1.4);
                            }
                        }
                        else
                        {
                            motion("9");
                            if (usePenaltyStrategy)
                            {
                                useFollowSearchGoal = false;
                                useSearchGoal = false;
                            }
                            stateCondition = 20;
                        }
                    }
                    else
                    { // Defense
                        modeImu = 2;
                        if (Ball_X != -1 && Ball_Y != -1)
                        {
                            trackBall();
                        }

                        if (pickUp)
                        {
                            if (signIn)
                            {
                                motion("9");
                                setWaktu();
                                stateCondition = 50;
                            }
                            else
                            {
                                motion("0");
                                headMove(0.0, -1.4);
                            }
                        }
                        else
                        {
                            motion("0");
                            setWaktu();
                            stateCondition = 130;
                        }
                    }
                    break;

                case 3: // Backup robot di kiri/kanan - Defense belakang di kiri/kanan
                    modeImu = 1;
                    if (KickOff == barelang_color || KickOff == dropball)
                    { // Attack
                        if (Ball_X != -1 && Ball_Y != -1)
                        {
                            trackBall();
                        }

                        if (pickUp)
                        {
                            if (signIn)
                            {
                                motion("9");
                                setWaktu();
                                stateCondition = 50;
                            }
                            else
                            {
                                motion("0");
                                headMove(0.0, -1.4);
                            }
                        }
                        else
                        {
                            motion("9");
                            if (usePenaltyStrategy)
                            {
                                useFollowSearchGoal = false;
                                useSearchGoal = false;
                            }
                            stateCondition = 30;
                        }
                    }
                    else
                    { // Defense

                        if (Ball_X != -1 && Ball_Y != -1)
                        {
                            trackBall();
                        }

                        if (pickUp)
                        {
                            if (signIn)
                            {
                                motion("9");
                                setWaktu();
                                stateCondition = 50;
                            }
                            else
                            {
                                motion("0");
                                headMove(0.0, -1.4);
                            }
                        }
                        else
                        {
                            motion("0");
                            setWaktu();
                            stateCondition = 140;
                        }
                    }
                    break;

                case 4: // cek Vision
                    stateCondition = 150;
                    break;

                default:
                    break;
                }
                break;

            case 1: // searching ball
                if (Ball_X == -1 && Ball_Y == -1 && !tracked)
                {
                    delayWaitBall = 0;
                    if (useLocalization)
                    {
                        if (robotNumber == 1)
                        {
                            normalSearchBallGrid(39, 0, -25, 21, 0, -25);
                        }
                        else if (robotNumber == 3)
                        {
                            normalSearchBallGrid(40, 0, -25, 22, 0, -25);
                        }
                        else if (robotNumber == 4)
                        {
                            normalSearchBallGrid(39, 0, 25, 21, 0, 25);
                        }
                        else if (robotNumber == 5)
                        {
                            normalSearchBallGrid(40, 0, 25, 22, 0, 25);
                        }
                        // normalSearchBallGrid(39,0,50, 15,0,50);
                        // normalSearchBall();
                    }
                    else
                    {
                        // motion("0");
                        normalSearchBall();
                    }
                }
                else
                {
                    tracked = true;
                }

                if (tracked)
                {
                    if (ballLost(20))
                    {
                        tracked = false;
                    }
                    else
                    {
                        trackBall();
                        if (delayWaitBall > 30)
                        { // 30
                            motion("9");
                            if (headTilt >= -1.8 && headPan >= -0.1 && headPan <= 0.1)
                            {
                                resetCase2();
                                stateCondition = 2;
                            }
                            else
                            {
                                followBall(0);
                            }
                        }
                        else
                        {
                            // motion("0");    //tes
                            // Walk(0.0, 0.0, 0.0);
                            delayWaitBall++;
                        } // printf("  delayWaitBall = %d,", delayWaitBall);
                    }
                }
                break;

            case 2: // follow, search goal dan coor
                // motion("9");
                cout << "masuk case 2" << endl;
                if (useCoordination)
                {
                    if (robotNumber == 1)
                    {
                        if ( // jika ada robot lain yang sudah masuk case eksekusi
                            (robot2State == 232 || robot2State == 7 || robot2State == 3 || robot2State == 8 || robot2State == 4 || robot2State == 5 || robot2State == -10 || robot2State == 10 || robot2State == 20 || robot2State == 30) ||
                            (robot3State == 232 || robot3State == 7 || robot3State == 3 || robot3State == 8 || robot3State == 4 || robot3State == 5 || robot3State == -10 || robot3State == 10 || robot3State == 20 || robot3State == 30) ||
                            (robot4State == 232 || robot4State == 7 || robot4State == 3 || robot4State == 8 || robot4State == 4 || robot4State == 5 || robot4State == -10 || robot4State == 10 || robot4State == 20 || robot4State == 30) ||
                            (robot5State == 232 || robot5State == 7 || robot5State == 3 || robot5State == 8 || robot5State == 4 || robot5State == 5 || robot5State == -10 || robot5State == 10 || robot5State == 20 || robot5State == 30))
                        {
                            exeCutor = false;
                        }
                        else if ( // jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
                            ((headTilt * -100) < robot2DBall) &&
                            ((headTilt * -100) < robot3DBall) &&
                            ((headTilt * -100) < robot4DBall) &&
                            ((headTilt * -100) < robot5DBall))
                        {
                            exeCutor = true;
                        }

                        if (exeCutor)
                        { // printf("\n  MyTurn......................\n");
                            // selain nomer robotNumber, false semua ...
                            // robot3exeCutor = false;
                            // delayWalkWaiting = 0;
                            // refreshMoveGrid();
                            myTurn();
                        }
                        else
                        { // printf("\n  YourTurn......................\n");
                            // if (robot3State == 232 || robot3State == 7 || robot3State == 3 || robot3State == 8 || robot3State == 4 || robot3State == 5 || robot3State == -10 || robot3State == 10 || robot3State == 20 || robot3State == 30) {
                            //	robot3exeCutor = true;
                            // }
                            waitingTurn();
                        }
                    }
                    else if (robotNumber == 2)
                    {
                        if ( // jika ada robot lain yang sudah masuk case eksekusi
                            (robot1State == 232 || robot1State == 7 || robot1State == 3 || robot1State == 8 || robot1State == 4 || robot1State == 5 || robot1State == -10 || robot1State == 10 || robot1State == 20 || robot1State == 30) ||
                            (robot3State == 232 || robot3State == 7 || robot3State == 3 || robot3State == 8 || robot3State == 4 || robot3State == 5 || robot3State == -10 || robot3State == 10 || robot3State == 20 || robot3State == 30) ||
                            (robot4State == 232 || robot4State == 7 || robot4State == 3 || robot4State == 8 || robot4State == 4 || robot4State == 5 || robot4State == -10 || robot4State == 10 || robot4State == 20 || robot4State == 30) ||
                            (robot5State == 232 || robot5State == 7 || robot5State == 3 || robot5State == 8 || robot5State == 4 || robot5State == 5 || robot5State == -10 || robot5State == 10 || robot5State == 20 || robot5State == 30))
                        {
                            exeCutor = false;
                        }
                        else if ( // jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
                            ((headTilt * -100) < robot1DBall) &&
                            ((headTilt * -100) < robot3DBall) &&
                            ((headTilt * -100) < robot4DBall) &&
                            ((headTilt * -100) < robot5DBall))
                        {
                            exeCutor = true;
                        }

                        if (exeCutor)
                        { // printf("\n  MyTurn......................\n");
                            myTurn();
                        }
                        else
                        { // printf("\n  YourTurn......................\n");
                            waitingTurn();
                        }
                    }
                    else if (robotNumber == 3)
                    {
                        if ( // jika ada robot lain yang sudah masuk case eksekusi
                            (robot1State == 232 || robot1State == 7 || robot1State == 3 || robot1State == 8 || robot1State == 4 || robot1State == 5 || robot1State == -10 || robot1State == 10 || robot1State == 20 || robot1State == 30) ||
                            (robot2State == 232 || robot2State == 7 || robot2State == 3 || robot2State == 8 || robot2State == 4 || robot2State == 5 || robot2State == -10 || robot2State == 10 || robot2State == 20 || robot2State == 30) ||
                            (robot4State == 232 || robot4State == 7 || robot4State == 3 || robot4State == 8 || robot4State == 4 || robot4State == 5 || robot4State == -10 || robot4State == 10 || robot4State == 20 || robot4State == 30) ||
                            (robot5State == 232 || robot5State == 7 || robot5State == 3 || robot5State == 8 || robot5State == 4 || robot5State == 5 || robot5State == -10 || robot5State == 10 || robot5State == 20 || robot5State == 30))
                        {
                            exeCutor = false;
                        }
                        else if ( // jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
                            ((headTilt * -100) < robot1DBall) &&
                            ((headTilt * -100) < robot2DBall) &&
                            ((headTilt * -100) < robot4DBall) &&
                            ((headTilt * -100) < robot5DBall))
                        {
                            exeCutor = true;
                        }

                        if (exeCutor)
                        { // printf("\n  MyTurn......................\n");
                            // selain nomer robotNumber, false semua ...
                            //  robot1exeCutor = false;
                            //  delayWalkWaiting = 0;
                            //  refreshMoveGrid();
                            myTurn();
                        }
                        else
                        { // printf("\n  YourTurn......................\n");
                            // if (robot1State == 232 || robot1State == 7 || robot1State == 3 || robot1State == 8 || robot1State == 4 || robot1State == 5 || robot1State == -10 || robot1State == 10 || robot1State == 20 || robot1State == 30) {
                            // 	robot1exeCutor = true;
                            // }
                            waitingTurn();
                        }
                    }
                    else if (robotNumber == 4)
                    {
                        if ( // jika ada robot lain yang sudah masuk case eksekusi
                            (robot1State == 232 || robot1State == 7 || robot1State == 3 || robot1State == 8 || robot1State == 4 || robot1State == 5 || robot1State == -10 || robot1State == 10 || robot1State == 20 || robot1State == 30) ||
                            (robot2State == 232 || robot2State == 7 || robot2State == 3 || robot2State == 8 || robot2State == 4 || robot2State == 5 || robot2State == -10 || robot2State == 10 || robot2State == 20 || robot2State == 30) ||
                            (robot3State == 232 || robot3State == 7 || robot3State == 3 || robot3State == 8 || robot3State == 4 || robot3State == 5 || robot3State == -10 || robot3State == 10 || robot3State == 20 || robot3State == 30) ||
                            (robot5State == 232 || robot5State == 7 || robot5State == 3 || robot5State == 8 || robot5State == 4 || robot5State == 5 || robot5State == -10 || robot5State == 10 || robot5State == 20 || robot5State == 30))
                        {
                            exeCutor = false;
                        }
                        else if ( // jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
                            ((headTilt * -100) < robot1DBall) &&
                            ((headTilt * -100) < robot2DBall) &&
                            ((headTilt * -100) < robot3DBall) &&
                            ((headTilt * -100) < robot5DBall))
                        {
                            exeCutor = true;
                        }

                        if (exeCutor)
                        { // printf("\n  MyTurn......................\n");
                            myTurn();
                        }
                        else
                        { // printf("\n  YourTurn......................\n");
                            waitingTurn();
                        }
                    }
                    else if (robotNumber == 5)
                    {
                        if ( // jika ada robot lain yang sudah masuk case eksekusi
                            (robot1State == 232 || robot1State == 7 || robot1State == 3 || robot1State == 8 || robot1State == 4 || robot1State == 5 || robot1State == -10 || robot1State == 10 || robot1State == 20 || robot1State == 30) ||
                            (robot2State == 232 || robot2State == 7 || robot2State == 3 || robot2State == 8 || robot2State == 4 || robot2State == 5 || robot2State == -10 || robot2State == 10 || robot2State == 20 || robot2State == 30) ||
                            (robot3State == 232 || robot3State == 7 || robot3State == 3 || robot3State == 8 || robot3State == 4 || robot3State == 5 || robot3State == -10 || robot3State == 10 || robot3State == 20 || robot3State == 30) ||
                            (robot4State == 232 || robot4State == 7 || robot4State == 3 || robot4State == 8 || robot4State == 4 || robot4State == 5 || robot4State == -10 || robot4State == 10 || robot4State == 20 || robot4State == 30))
                        {
                            exeCutor = false;
                        }
                        else if ( // jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
                            ((headTilt * -100) < robot1DBall) &&
                            ((headTilt * -100) < robot2DBall) &&
                            ((headTilt * -100) < robot3DBall) &&
                            ((headTilt * -100) < robot4DBall))
                        {
                            exeCutor = true;
                        }

                        if (exeCutor)
                        { // printf("\n  MyTurn......................\n");
                            myTurn();
                        }
                        else
                        { // printf("\n  YourTurn......................\n");
                            waitingTurn();
                        }
                    }
                }
                else
                {
                    myTurn();
                }
                break;

            case 7: // imu
                motion("9");
                // printf("  MYTURN MYTURN MYTURN MYTURN !!!!\n");
                // motion("0");
                if (ballLost(20))
                {
                    if (tunggu > 40)
                    {
                        resetCase1();
                        stateCondition = 1;
                    }
                    else
                    {
                        tiltSearchBall(0.0);
                    }
                    tunggu++;
                    Walk(0.0, 0.0, 0.0);
                }
                else
                {
                    trackBall();
                    tunggu = 0;

                    if (useImu)
                    {
                        if (robotDirection && headPan >= -0.2 && headPan <= 0.2)
                        { // 0.4
                            // Walk(0.0, 0.0, 0.0);

                            if (useDribble == true || useSearchGoal == true)
                            {
                                resetCase3();
                                stateCondition = 3;
                            }
                            else
                            {
                                resetCase5();
                                stateCondition = 5;
                            }
                        }
                        else
                        {
                            if (headTilt >= cAktif && headPan >= -0.2 && headPan <= 0.2)
                            { // 0.4
                                if (delay > 5)
                                {
                                    cekWaktu(20);
                                    if (timer)
                                    {
                                        robotDirection = true;
                                    }
                                    else
                                    {
                                        if (useSideKick)
                                        {
                                            printf("....use side kick\n");
                                            if (useLocalization)
                                            {
                                                if (Grid >= 1 && Grid <= 42)
                                                {
                                                    if (angle >= 45)
                                                    {                                 // 45
                                                        modeKick = 4;                 // tendangSamping
                                                        Imu(90 + outGrid, cSekarang); // 90
                                                    }
                                                    else if (angle <= -45)
                                                    {                                  //-45
                                                        modeKick = 3;                  // tendangSamping
                                                        Imu(-90 + outGrid, cSekarang); // 90
                                                    }
                                                    else
                                                    {
                                                        modeKick = tendangJauh;
                                                        Imu(outGrid, cSekarang);
                                                    }
                                                }
                                                else if (Grid >= 43 && Grid <= 54)
                                                {
                                                    if (robotPos_Y > 0)
                                                    { // posisi Y dari 0 - 300
                                                        if (angle >= 20 && angle <= 180)
                                                        {                                 // 45
                                                            modeKick = 4;                 // tendangSamping
                                                            Imu(90 + outGrid, cSekarang); // 90
                                                        }
                                                        else
                                                        {
                                                            modeKick = tendangJauh;
                                                            Imu(outGrid, cSekarang);
                                                        }
                                                    }
                                                    else
                                                    { // posisi Y dari -300 - 0
                                                        if (angle <= -20 && angle >= -180)
                                                        {                                  //-45
                                                            modeKick = 3;                  // tendangSamping
                                                            Imu(-90 + outGrid, cSekarang); // 90
                                                        }
                                                        else
                                                        {
                                                            modeKick = tendangJauh;
                                                            Imu(outGrid, cSekarang);
                                                        }
                                                    }
                                                }
                                            }
                                            else
                                            {
                                                printf("....rotate to imu\n");
                                                if (angle >= 50)
                                                {                       // 45
                                                    modeKick = 4;       // tendangSamping
                                                    Imu(75, cSekarang); // 90
                                                }
                                                else if (angle <= -50)
                                                {                        //-45
                                                    modeKick = 3;        // tendangSamping
                                                    Imu(-75, cSekarang); //-90
                                                }
                                                else
                                                {
                                                    modeKick = tendangJauh;
                                                    Imu(0, cSekarang);
                                                }
                                            }
                                        }
                                        else
                                        {
                                            modeKick = tendangJauh;
                                            if (useLocalization)
                                            {
                                                Imu(outGrid, cSekarang);
                                            }
                                            else
                                            {
                                                Imu(0, cSekarang);
                                            }
                                        }
                                    }
                                }
                                else
                                {
                                    setWaktu();
                                    robotDirection = false;
                                    delay++;
                                }
                            }
                            else
                            {
                                delay = 0;
                                followBall(0);
                            }
                        }
                    }
                    else
                    {
                        if (headTilt >= (cSekarang - 0.3))
                        { // 0.2
                            // Walk(0.0, 0.0, 0.0);

                            if (useDribble == true || useSearchGoal == true)
                            {
                                resetCase3();
                                stateCondition = 3;
                            }
                            else
                            {
                                resetCase5();
                                stateCondition = 5;
                            }
                        }
                        else
                        {
                            if (useLocalization && useUpdateCoordinate)
                            {
                                updateCoordinatFromVision();
                            }
                            followBall(0);
                        }
                    }
                }
                break;

            case 3: // dribble
                motion("9");

                if (ballLost(20))
                {
                    if (tunggu > 40)
                    {
                        resetCase1();
                        stateCondition = 1;
                    }
                    else
                    {
                        tiltSearchBall(0.0);
                    }
                    tunggu++;
                    Walk(0.0, 0.0, 0.0);
                }
                else
                {
                    trackBall();
                    tunggu = 0;

                    if (useDribble)
                    {
                        if (countDribble > 300)
                        { // 300 //printf("ke state SearchGoal....................................................................\n\n\n");
                            resetCase8();
                            stateCondition = 8;
                        }
                        else
                        {
                            // dribble(sudutTengah, 0.15); //0.14 //0.05 //0.08 //arah imu, speed; speed = speed * 0.3
                            dribble(outGrid, 0.15); // 0.14 //0.05 //0.08 //arah imu, speed; speed = speed * 0.3
                            countDribble++;
                        }
                    }
                    else
                    {
                        Walk(0.0, 0.0, 0.0);

                        if (useSearchGoal == true)
                        {
                            resetCase4();
                            stateCondition = 4;
                        }
                        else
                        {
                            resetCase5();
                            stateCondition = 5;
                        }
                    }
                }
                break;

            case 8: // Imu lagi setelah dribble
                motion("9");

                if (ballLost(20))
                {
                    if (tunggu > 40)
                    {
                        resetCase1();
                        stateCondition = 1;
                    }
                    else
                    {
                        tiltSearchBall(0.0);
                        Walk(0.0, 0.0, 0.0);
                    }
                    tunggu++;
                }
                else
                {
                    trackBall();
                    tunggu = 0;

                    if (useImu)
                    {
                        if (robotDirection && headPan >= -0.4 && headPan <= 0.4)
                        {
                            Walk(0.0, 0.0, 0.0);
                            resetCase4();
                            stateCondition = 4;
                        }
                        else
                        {
                            if (headTilt >= cAktif && headPan >= -0.4 && headPan <= 0.4)
                            {
                                modeKick = tendangJauh;
                                Imu(outGrid, cSekarang);
                            }
                            else
                            {
                                robotDirection = false;
                                followBall(0);
                            }
                        }
                    }
                    else
                    {
                        if (headTilt >= (cSekarang - 0.2) && headPan >= -0.4 && headPan <= 0.4)
                        {
                            Walk(0.0, 0.0, 0.0);
                            resetCase4();
                            stateCondition = 4;
                        }
                        else
                        {
                            followBall(0);
                        }
                    }
                }
                break;

            case 4: // Search Goal
                motion("9");

                if (useSearchGoal)
                {
                    printf("cariGawang\n\n");
                    if (rotateGoal == true)
                    {
                        Walk(0.0, 0.0, 0.0);
                        if (dribbleOnly)
                        {
                            resetCase3();
                            stateCondition = 3; // dribble
                        }
                        else
                        {
                            // printf("ke case 5\n\n");
                            resetCase5();
                            stateCondition = 5; // kick
                        }
                    }
                    else
                    {
                        rotateToGoal(1); // 1
                    }
                }
                else
                { // printf("tidakCariGawang\n\n");
                    Walk(0.0, 0.0, 0.0);
                    if (dribbleOnly)
                    {
                        resetCase3();
                        stateCondition = 3; // dribble
                    }
                    else
                    {
                        resetCase5();
                        stateCondition = 5; // kick
                    }
                }
                break;

            case 5: // Kick
                // motion("9");
                if (ballLost(30))
                {
                    if (tunggu > 40)
                    {
                        resetCase1();
                        stateCondition = 1;
                    }
                    else
                    {
                        tiltSearchBall(0.0);
                        Walk(0.0, 0.0, 0.0);
                    }
                    tunggu++;
                }
                else
                {
                    trackBall();

                    if (tendang)
                    {
                        resetCase6();
                        stateCondition = 6;
                    }
                    else
                    {
                        kick(modeKick);
                    }
                }
                break;

            case 6: // Search After Kick
                motion("9");

                if (useLocalization && useUpdateCoordinate)
                {
                    updateCoordinatFromVision();
                }
                if (tunggu > 5)
                {                // kedua
                    cekWaktu(3); // 4

                    if (timer)
                    {
                        if (ballLost(20))
                        {
                            confirmsBall = 0;
                            waitTracking = 0;

                            if (searchRectangle)
                            {
                                if (useCoordination)
                                {
                                    semeh = 232;
                                    sendRobotCoordinationData(robotNumber, robotStatus, stateCondition, Grid, 0, semeh, GridBall, backIn);
                                }

                                if (sumTilt > 300)
                                {
                                    resetCase1();
                                    stateCondition = 1;
                                }
                                else
                                {
                                    searchBallRectang(-1.5, -1.6, -0.8, 1.6);
                                    sumTilt++;
                                }

                                if (modeKick == 3 || modeKick == 4)
                                {                                              // sideKick
                                    jalanDirection(kejar, 0.0, lastDirection); // saveAngle
                                }
                                else
                                {
                                    Walk(kejar, 0.0, 0.0);
                                }
                            }
                            else
                            {
                                if (useCoordination)
                                {
                                    semeh = 1;
                                    sendRobotCoordinationData(robotNumber, robotStatus, 232, Grid, 1, semeh, GridBall, backIn);
                                }
                                Walk(0.0, 0.0, 0.0);

                                if (tunda > 10)
                                {
                                    SearchBall(3); // first Search
                                    if (posPan > 1.5 && countTilt == 1)
                                    {
                                        tiltRate = 0.05;
                                        panRate = 0.05;
                                        searchRectangle = true;
                                    }
                                }
                                else
                                {
                                    posPan = 1.45;
                                    posTilt = -0.8;
                                    tiltRate = -0.05;
                                    panRate = -0.05;
                                    headMove(posPan, posTilt);
                                    tunda++;
                                }
                            }
                        }
                        else
                        {
                            trackBall();

                            if (waitTracking > 30)
                            {
                                searchRectangle = true;
                            }
                            else
                            {
                                tiltRate = 0.05;
                                panRate = 0.05;
                                waitTracking++;
                            }

                            if (useCoordination)
                            {
                                sendRobotCoordinationData(robotNumber, robotStatus, stateCondition, Grid, 1, semeh, GridBall, backIn);
                                backToCoordinations();
                            }

                            if (confirmsBall > 30)
                            {
                                if (headTilt >= -0.7 && searchRectangle == false)
                                { // langsung Tendang lagi
                                    Walk(0.0, 0.0, 0.0);
                                    resetCase5();
                                    stateCondition = 5;
                                }
                                else
                                {
                                    // if (headTilt >= -1.5 && headPan >= -0.2 && headPan <= 0.2) {
                                    // if (headTilt >= -1.8 && headPan >= -0.2 && headPan <= 0.2) {
                                    if (headTilt >= -1.6 && headPan >= -0.2 && headPan <= 0.2)
                                    {
                                        resetCase2();
                                        stateCondition = 2;
                                    }
                                    else
                                    {
                                        followBall(0);
                                    }
                                }
                            }
                            else
                            {
                                Walk(0.0, 0.0, 0.0);
                                confirmsBall++;
                            }
                        }
                    }
                    else
                    {
                        if (useCoordination)
                        {
                            semeh = 1;
                            sendRobotCoordinationData(robotNumber, robotStatus, 232, Grid, 1, semeh, GridBall, backIn);
                        }
                        // Walk(0.0, 0.0, 0.0);

                        if (modeKick == 3)
                        { // tendangSamping
                            posPan = -1.6;
                            posTilt = -1.6;
                            headMove(posPan, posTilt);
                        }
                        else if (modeKick == 4)
                        { // tendangSamping
                            posPan = 1.6;
                            posTilt = -1.6;
                            headMove(posPan, posTilt);
                        }
                        else
                        { // tendangDepan
                            posPan = 0.0;
                            posTilt = -1.6;
                            headMove(posPan, posTilt);
                        }
                        countTilt = 0;
                        sumTilt = 0;
                        tunda = 0;
                        confirmsBall = 0;
                        waitTracking = 0;
                    }
                }
                else
                {               // pertama
                    setWaktu(); //``
                    tunggu++;
                }
                break;

            case -20: // serang lurus -> dribble
                if (ballLost(20))
                {
                    if (tunggu > 40)
                    {
                        resetCase1();
                        stateCondition = 1;
                    }
                    else
                    {
                        tiltSearchBall(0.0);
                        Walk(0.0, 0.0, 0.0);
                    }
                    tunggu++;
                    delay = 0;
                }
                else
                {
                    trackBall();
                    tunggu = 0;

                    if (robotDirection && headPan >= -0.4 && headPan <= 0.4)
                    {
                        // if (headTilt >= (cSekarang - 0.2)) {
                        if (delay > 5)
                        {
                            cekWaktu(10);
                            if (timer)
                            {
                                resetCase4();
                                stateCondition = 4;
                            }
                            else
                            {
                                dribble(sudutTengah, 0.15);
                                // followBall(0);
                            }
                        }
                        else
                        {
                            setWaktu();
                            delay++;
                        }
                        //} else {
                        //	if (headTilt >= -1.0) {
                        //		ballPositioning(0.0, cSekarang, 0.12);
                        //	} else {
                        //		followBall(0);
                        //	}
                        //}
                    }
                    else
                    {
                        if (headTilt >= cAktif && headPan >= -0.8 && headPan <= 0.8)
                        {
                            Imu(sudutTengah, cSekarang);
                        }
                        else
                        {
                            robotDirection = false;
                            followBall(0);
                        }
                    }
                }
                break;

            case -10: // serang lurus -> eksekusi tendang
                if (ballLost(20))
                {
                    if (tunggu > 40)
                    {
                        resetCase1();
                        stateCondition = 1;
                    }
                    else
                    {
                        tiltSearchBall(0.0);
                        Walk(0.0, 0.0, 0.0);
                    }
                    tunggu++;
                }
                else
                {
                    trackBall();
                    tunggu = 0;

                    if (tendang)
                    {
                        Walk(0.0, 0.0, 0.0);
                        resetCase6();
                        stateCondition = 6;
                    }
                    else
                    {
                        // kickNoSudut(6);
                        // rotateKickOff(0.0, tendangDekat);
                        rotateKickOffImu(0, tendangDekat);
                        // kick(tendangOper);
                    }
                }
                break;

            case 10: // serang ke kanan
                if (ballLost(20))
                {
                    if (tunggu > 40)
                    {
                        resetCase1();
                        stateCondition = 1;
                    }
                    else
                    {
                        tiltSearchBall(0.0);
                        Walk(0.0, 0.0, 0.0);
                    }
                    tunggu++;
                }
                else
                {
                    trackBall();
                    tunggu = 0;

                    if (tendang)
                    {
                        Walk(0.0, 0.0, 0.0);
                        resetCase6();
                        stateCondition = 6;
                    }
                    else
                    {
                        if (!usePenaltyStrategy)
                        {
                            rotateKickOff(-2, tendangDekat); //-2.5
                                                             // rotateKickOffImu(30, tendangDekat);
                        }
                        else
                        {
                            rotateKickOff(-2, tendangDekat); //-2.5
                                                             // rotateKickOffImu(25, tendangJauh);
                        }
                    }
                }
                break;

            case 20: // serang ke kiri
                if (ballLost(20))
                {
                    if (tunggu > 40)
                    {
                        resetCase1();
                        stateCondition = 1;
                    }
                    else
                    {
                        tiltSearchBall(0.0);
                        Walk(0.0, 0.0, 0.0);
                    }
                    tunggu++;
                }
                else
                {
                    trackBall();
                    tunggu = 0;

                    if (tendang)
                    {
                        Walk(0.0, 0.0, 0.0);
                        resetCase6();
                        stateCondition = 6;
                    }
                    else
                    {
                        if (!usePenaltyStrategy)
                        {
                            rotateKickOff(2, tendangDekat); // 2.5
                            // rotateKickOffImu(-30, tendangDekat);
                        }
                        else
                        {
                            rotateKickOff(2, tendangDekat); // 2.5
                            // rotateKickOffImu(-25, tendangJauh);
                        }
                    }
                }
                break;

            case 30: // Backup kiri-kanan
                if (ballLost(20))
                {
                    delay = 0;
                    if (tunggu > 40)
                    {
                        resetCase1();
                        stateCondition = 1;
                    }
                    else
                    {
                        tiltSearchBall(0.0);
                        Walk(0.0, 0.0, 0.0);
                    }
                    tunggu++;
                }
                else
                {
                    trackBall();
                    tunggu = 0;

                    if (useCoordination)
                    {
                        backToCoordinations();
                    }

                    if (delay > 30)
                    {
                        // if (headTilt >= -1.5 && headPan >= -0.2 && headPan <= 0.2) {
                        if (headTilt >= -1.8 && headPan >= -0.2 && headPan <= 0.2)
                        {
                            // if (headTilt >= -1.6 && headPan >= -0.2 && headPan <= 0.2) {
                            resetCase2();
                            stateCondition = 2;
                        }
                        else
                        {
                            followBall(0);
                        }
                    }
                    else
                    {
                        Walk(0.0, 0.0, 0.0);
                        delay++;
                    }
                }
                break;

            case 50: // pickup
                if (backIn)
                {
                    countInitPos = 0;
                }
                else
                {
                    countInitPos = 1;
                } // lock initial pos
                cekWaktu(30);
                if (second > 5)
                { // 5
                    backIn = false;
                    if (Ball_X == -1 && Ball_Y == -1 && !tracked)
                    {
                        delay = 0;

                        if (timer)
                        {
                            Walk(0.0, 0.0, 0.0);
                            resetCase1();
                            stateCondition = 1;
                        }
                        else
                        {
                            threeSearchBall();
                            if (second <= 15)
                            {
                                Walk(kejar, 0.0, 0.0);
                            }
                            else
                            {
                                jalanDirection(kejar, 0.0, 0); // XYA
                            }
                        }
                    }
                    else
                    {
                        tracked = true;
                    }

                    if (tracked)
                    {
                        if (ballLost(20))
                        {
                            tracked = false;
                        }
                        else
                        {
                            trackBall();

                            if (useCoordination)
                            {
                                backToCoordinations();
                            }

                            if (delay > 30)
                            {
                                // if (headTilt >= -1.5 && headPan >= -0.2 && headPan <= 0.2) {
                                if (headTilt >= -1.8 && headPan >= -0.2 && headPan <= 0.2)
                                {
                                    // if (headTilt >= -1.6 && headPan >= -0.2 && headPan <= 0.2) {
                                    resetCase2();
                                    stateCondition = 2;
                                }
                                else
                                {
                                    followBall(0);
                                }
                            }
                            else
                            {
                                Walk(0.0, 0.0, 0.0);
                                delay++;
                            }
                        }
                    }
                }
                else
                {
                    if (second > 2)
                    {
                        tiltSearchBall(0.0);
                    }
                    else
                    {
                        predictGoal(angle, -1.4);
                    }
                    Walk(kejar, 0.0, 0.0);
                }
                break;

            case 130: // defense didepan
                printf("defense di depan\n\n");
                cekWaktu(10);
                if (second < 3)
                {
                    ball_panKP = 0.05;
                    ball_panKD = 0.0000755;
                    ball_tiltKP = 0.05;
                    ball_tiltKD = 0.0000755;
                }

                if (timer)
                {
                    motion("9");

                    getParameters(); // robot parameters
                    resetCase2();
                    stateCondition = 2;
                }
                else
                {
                    if (Ball_X == -1 && Ball_Y == -1 && !tracked)
                    {
                        if (countDef > 5)
                        {
                            normalSearchBall();
                        }
                        else
                        {
                            resetCase1();
                            countDef++;
                        }
                    }
                    else
                    {
                        tracked = true;
                    }

                    if (tracked)
                    {
                        if (ballLost(30))
                        {
                            tracked = false;
                        }
                        else
                        {
                            trackBall();

                            if (tunggu >= 100)
                            {
                                hitungGerakBola();
                                if (kondisiBola == 0)
                                {
                                    motion("0");
                                }
                                else
                                {
                                    motion("9");

                                    getParameters(); // robot parameters
                                    resetCase2();
                                    stateCondition = 2;
                                }
                            }
                            else
                            {
                                motion("0");
                                tunggu++;
                            }
                        }
                    }
                }
                break;

            case 140: // defense disamping
                printf("defense disamping \n\n");
                cekWaktu(10);
                if (second < 3)
                {
                    ball_panKP = 0.05;
                    ball_panKD = 0.0000755;
                    ball_tiltKP = 0.05;
                    ball_tiltKD = 0.0000755;
                }

                if (timer)
                {
                    motion("9");

                    getParameters(); // robot parameters
                    resetCase2();
                    stateCondition = 2;
                }
                else
                {
                    if (Ball_X == -1 && Ball_Y == -1 && !tracked)
                    {
                        if (countDef > 5)
                        {
                            normalSearchBall();
                        }
                        else
                        {
                            resetCase1();
                            countDef++;
                        }
                    }
                    else
                    {
                        tracked = true;
                    }

                    if (tracked)
                    {
                        if (ballLost(30))
                        {
                            tracked = false;
                        }
                        else
                        {
                            trackBall();

                            if (tunggu >= 100)
                            {
                                hitungGerakBola();
                                if (kondisiBola == 0)
                                {
                                    motion("0");
                                }
                                else
                                {
                                    motion("9");

                                    getParameters(); // robot parameters
                                    resetCase2();
                                    stateCondition = 2;
                                }
                            }
                            else
                            {
                                motion("0");
                                tunggu++;
                            }
                        }
                    }
                }
                break;

            case 150: // setting receler
                break;

            case 200: // heandle error Imu
                if (msg_yaw == 0)
                {
                    zeroState = true;

                    motion("0");
                    headMove(batasKanan, batasAtas);
                }
                else
                {
                    zeroState = false;

                    resetCase1();
                    stateCondition = 1;
                }
                break;

            case 1000:
                // normalSearchBall();
                motion("9");
                // predictGoalTeam(180, posTiltGoal);
                Walk(0.08, 0.0, 0.0);
                // if (goalLost(20)) {
                // threeSearchBall();
                // searchBallRectang(-1.5, -1.6, -0.8, 1.6);
                // tiltSearchBall(0.0);
                //	panSearchBall(-1.6);
                // headMove(0.0, -1.8);
                //} else {
                trackGoal();
                //	updateCoordinatFromVision();
                //}
                // kalkulasiJarakGawang(Goal_LD, Goal_RD); //jarak menjadi waktu
                break;

            case 1001: // untuk ambil data odometry per langkahnya
                headMove(0.0, -1.6);
                // headMove(0.0,-1.5);
                getServPos();

                if (walkX < 0)
                {
                    printf(" Jumlah Mundur = %d\n", walkTotMun);
                }
                else if (walkX > 0)
                {
                    printf(" Jumlah Maju = %d\n", walkTot);
                }
                if (walkY > 0)
                {
                    printf(" Jumlah Kiri = %d\n", csmKiri);
                }
                else if (walkY < 0)
                {
                    printf(" Jumlah Kanan = %d\n", csmKanan);
                }
                // if(strategyCM == 0){

                // 		if (csmKiri >= 10) {
                // 			motion("0");
                // 		// if (varCount >= 200) {
                // 		// 	motion("0");
                // 			//if ( varCount > 5 ) {
                // 			//	cekWaktu(3);
                // 			//	if (timer) {
                // 			//		motion("7");
                // 			//	}
                // 			//} else {
                // 			//	setWaktu();
                // 			//	varCount++;
                // 			//}
                // 		} else {
                // 			motion("9");
                // 			// if(strategyCM == 0){
                // 			// 	Walk(0.0,-0.03,0.0);
                // 			// }else if(strategyCM == 1){
                // 				Walk(0.0,0.03,0.0);
                // 			// }else if(strategyCM == 2){
                // 			// 	Walk(0.0,0.03,0.0);
                // 			// }

                // 			// varCount++;
                // 		}
                // }else if(strategyCM == 1){

                // 		if (csmKanan >= 10) {
                // 			motion("0");
                // 		// if (varCount >= 200) {
                // 		// 	motion("0");
                // 			//if ( varCount > 5 ) {
                // 			//	cekWaktu(3);
                // 			//	if (timer) {
                // 			//		motion("7");
                // 			//	}
                // 			//} else {
                // 			//	setWaktu();
                // 			//	varCount++;
                // 			//}
                // 		} else {
                // 			motion("9");
                // 			Walk(0.0,-0.03,0.0);

                // 			// varCount++;
                // 		}

                // }

                if (csmKiri >= 15)
                {
                    motion("0");
                }
                else
                {
                    motion("9");
                    if (strategyCM == 0)
                    {
                        Walk(0.0, 0.01, 0.0);
                    }
                    else if (strategyCM == 1)
                    {
                        Walk(0.0, 0.02, 0.0);
                    }
                    else if (strategyCM == 2)
                    {
                        Walk(0.0, 0.03, 0.0);
                    }
                }

                // bacaKondisi(RobotWalk_X, RobotWalk_Y);
                // if (varCount > 200) {
                // 	motion("0");
                // } else {
                // 	Walk(0.0,0.03,0.0);
                // 	motion("9");
                // 	// printf("  Lleg(%.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf),", Lleg1, Lleg2, Lleg3, Lleg4, Lleg5, Lleg6);
                // 	// printf("  Rleg(%.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf),\n", Rleg1, Rleg2, Rleg3, Rleg4, Rleg5, Rleg6);
                // 	printf("  Lleg(%.2lf)\n", Lleg4);
                // 	varCount++;
                // }

                break;

            case 330:
                motion("9");
                Walk(0.0, 0.0, 0.0);
                break;

            case 331:

                break;

            default:
                break;
            }
        }
    }

    void readButton(const bfc_msgs::msg::Button::SharedPtr msg_btn_)
    {
        msg_strategy = msg_btn_->strategy;
        msg_kill = msg_btn_->kill;
    }

    void readImu(const sensor_msgs::msg::Imu::SharedPtr msg_imu_)
    {
        msg_roll = msg_imu_->angular_velocity.x;
        msg_pitch = msg_imu_->angular_velocity.y;
        msg_yaw = msg_imu_->angular_velocity.z;
    }

    void objCoor(const std_msgs::msg::Int32MultiArray::SharedPtr msg_bbox_)
    {
        Ball_X = msg_bbox_->data[0];
        Ball_Y = msg_bbox_->data[1];
        Goal_X = msg_bbox_->data[2];
        Goal_Y = msg_bbox_->data[3];
    }

    void readGameControllerData(const std_msgs::msg::Int64MultiArray::SharedPtr msg)
    {
        State = msg->data[0];
        FirstHalf = msg->data[1];
        Version = msg->data[2];
        PacketNumber = msg->data[3];
        PlayerTeam = msg->data[4];
        // GameTipe = msg->data[7];
        KickOff = msg->data[5];
        SecondaryState = msg->data[6];
        DropTeam = msg->data[7];
        DropTime = msg->data[8];
        Remaining = msg->data[9];
        SecondaryTime = msg->data[10];
        // ket :    1 = msg->data[]; untuk data GameController yang kiri
        //	        2 = msg->data[]; untuk data GameController yang kanan
        timNumber1 = msg->data[11];
        timNumber2 = msg->data[12];
        timColour1 = msg->data[13];
        timColour2 = msg->data[14];
        Score1 = msg->data[15];
        Score2 = msg->data[16];
        Penaltyshoot1 = msg->data[17];
        Penaltyshoot2 = msg->data[18];
        Singleshoot1 = msg->data[19];
        Singleshoot2 = msg->data[20];
        // Coachsequence1 = msg->data[24];
        // Coachsequence2 = msg->data[25];

        Penalty1 = msg->data[21];
        Penalty2 = msg->data[22];
        TimeUnpenalis1 = msg->data[23];
        TimeUnpenalis2 = msg->data[24];
        // YellowCard1 = msg->data[30];
        // YellowCard2 = msg->data[31];
        // RedCard1 = msg->data[32];
        // RedCard2 = msg->data[33];
    }

    // Head Movement =============================================================================
    void headMove(double pan, double tilt)
    {
        if (useRos)
        { // Socket
            auto msg_head_ = bfc_msgs::msg::HeadMovement();
            if (robotFall)
            {
                msg_head_.pan = 0.0;
                msg_head_.tilt = -2.1;
            }
            else
            {
                msg_head_.pan = ceil(pan * 100.0) / 100.0;
                msg_head_.tilt = ceil(tilt * 100.0) / 100.0;
            }
            cmd_head_->publish(msg_head_);
        }
        else
        { // NotePad
            FILE *fp, *outputfp;
            outputfp = fopen("../Player/HeadNote", "wb");
            fprintf(outputfp, "%.2lf,%.2lf", pan, tilt);
            fclose(outputfp);
        }

        headPan = posPan = ceil(pan * 100.0) / 100.0;
        headTilt = posTilt = ceil(tilt * 100.0) / 100.0;
        // printf("headPan = %.2f \t headTilt = %.2f\n", pan, tilt);
    }

    // Robot Movement ============================================================================
    void motion(char line[2])
    {
        // printf("Motion = %s\n",line[2]);
        // Tendang Jauh Kiri		= 1
        // Tendang Jauh Kanan		= 2

        // Tendang Pelan Kiri		= 3
        // Tendang Pelan Kanan		= 4

        // Tendang Ke Samping Kiri	= 5
        // Tendang Ke Samping Kanan	= 6

        // Tendang WalkKick Kiri		= t
        // Tendang WalkKick Kanan	= y

        // Duduk				= 7
        // Berdiri			= 8
        // Play				= 9
        // Stop				= 0

        if (useRos)
        { // Socket
            auto msg_mot_ = std_msgs::msg::String();
            msg_mot_.data = &line[0];
            cmd_mot_->publish(msg_mot_);
        }
        else
        { // NotePad
            FILE *outputfp;
            outputfp = fopen("../Player/WalkNote", "wb");
            fprintf(outputfp, "%s", &line[0]);
            fclose(outputfp);
        }

        // untuk kondisi offset imu
        if (line == "9")
        {
            robotGerak = true;
        }
        else if (line == "0")
        {
            robotGerak = false;
        }
    }

    // Walk Controller ===========================================================================
    double walkX = 0.0,
           walkY = 0.0,
           walkA = 0.0;
    double Walk(double x, double y, double a)
    {

        char line[50];

        walkX = x;
        walkY = y;
        walkA = a;

        RobotWalk_X = walkX; // + erorrXwalk;
        RobotWalk_Y = walkY; // + erorrYwalk;
        RobotWalk_A = walkA; // + erorrAwalk;

        if (robotNumber != 2 || robotNumber != 5)
        {
            if (RobotWalk_X >= 0.08)
            {
                RobotWalk_X = 0.08;
            }
            else if (RobotWalk_X <= -0.03)
            {
                RobotWalk_X = -0.03;
            }

            if (RobotWalk_Y >= 0.03)
            {
                RobotWalk_Y = 0.03;
            }
            else if (RobotWalk_Y <= -0.03)
            {
                RobotWalk_Y = -0.03;
            }

            if (RobotWalk_A >= 0.4)
            {
                RobotWalk_A = 0.4;
            }
            else if (RobotWalk_A <= -0.4)
            {
                RobotWalk_A = -0.4;
            }
        }
        else
        {
            if (RobotWalk_X >= 0.06)
            {
                RobotWalk_X = 0.06;
            }
            else if (RobotWalk_X <= -0.03)
            {
                RobotWalk_X = -0.03;
            }

            if (RobotWalk_Y >= 0.03)
            {
                RobotWalk_Y = 0.03;
            }
            else if (RobotWalk_Y <= -0.03)
            {
                RobotWalk_Y = -0.03;
            }

            if (RobotWalk_A >= 0.4)
            {
                RobotWalk_A = 0.4;
            }
            else if (RobotWalk_A <= -0.4)
            {
                RobotWalk_A = -0.4;
            }
        }

        if (useRos)
        {
            auto msg_walk_ = geometry_msgs::msg::Twist();
            msg_walk_.linear.x = x + erorrXwalk;
            msg_walk_.linear.y = y + erorrYwalk;
            msg_walk_.linear.z = a + erorrAwalk;
            cmd_vel_->publish(msg_walk_);
        }
        else
        { // NotePad
            FILE *outputfp;
            strcpy(line, "walk");
            outputfp = fopen("../Player/WalkNote", "wb");
            fprintf(outputfp, "%s,%.2lf,%.2lf,%.2lf", &line[0], x + erorrXwalk, y, a); // setting jalan ditempat default
            fclose(outputfp);
            // printf("walk : %g,%g,%g\n",x,y,a);
        }

        return (walkX, walkY, walkA);
    }

    void declareParameters()
    {
        this->declare_parameter("robotNumber", 0);
        this->declare_parameter("frame_X", 640);
        this->declare_parameter("frame_Y", 480);
        this->declare_parameter("pPanTendang", -0.20);
        this->declare_parameter("pTiltTendang", -0.57);
        this->declare_parameter("ballPositioningSpeed", 0.12);
        this->declare_parameter("cSekarang", -0.60);
        this->declare_parameter("cAktif", -1.40);
        this->declare_parameter("posTiltLocal", -1.90);
        this->declare_parameter("posTiltGoal", -1.80);
        this->declare_parameter("ball_panKP", 0.075);
        this->declare_parameter("ball_panKD", 0.0000505);
        this->declare_parameter("ball_tiltKP", 0.05);
        this->declare_parameter("ball_tiltKD", 0.0000755);
        this->declare_parameter("goal_panKP", 0.10);
        this->declare_parameter("goal_panKD", 0.000050);
        this->declare_parameter("goal_tiltKP", 0.05);
        this->declare_parameter("goal_tiltKD", 0.000050);
        this->declare_parameter("errorXwalk", 0.0);
        this->declare_parameter("errorYwalk", 0.0);
        this->declare_parameter("errorAwalk", 0.0);
        this->declare_parameter("jalan", 0.040);
        this->declare_parameter("lari", 0.050);
        this->declare_parameter("kejar", 0.060);
        this->declare_parameter("kejarMid", 0.070);
        this->declare_parameter("kejarMax", 0.089);
        this->declare_parameter("tendangJauh", 1);
        this->declare_parameter("tendangSamping", 3);
        this->declare_parameter("tendangDekat", 5);
        this->declare_parameter("sudutTengah", 0);
        this->declare_parameter("sudutKanan", 30);
        this->declare_parameter("sudutKiri", -30);
        this->declare_parameter("rotateGoal_x", -0.0005);
        this->declare_parameter("rotateGoal_y", 0.017);
        this->declare_parameter("rotateGoal_a", 0.14);
        this->declare_parameter("myAccrX", 0.3);
        this->declare_parameter("myAccrY", 0.0);
        this->declare_parameter("tinggiRobot", 48);
        this->declare_parameter("outputSudutY1", 9.0);
        this->declare_parameter("inputSudutY1", -0.40);
        this->declare_parameter("outputSudutY2", 60.0);
        this->declare_parameter("inputSudutY2", -1.43);
        this->declare_parameter("outputSudutX1", 0.0);
        this->declare_parameter("inputSudutX1", 0.0);
        this->declare_parameter("outputSudutX2", 45.0);
        this->declare_parameter("inputSudutX2", -0.8);
        this->declare_parameter("usePenaltyStrategy", false);
        this->declare_parameter("useVision", false);
        this->declare_parameter("useImu", false);
        this->declare_parameter("useRos", false);
        this->declare_parameter("useGameController", false);
        this->declare_parameter("useCoordination", false);
        this->declare_parameter("useLocalization", false);
        this->declare_parameter("useFollowSearchGoal", false);
        this->declare_parameter("useSearchGoal", false);
        this->declare_parameter("useDribble", false);
        this->declare_parameter("dribbleOnly", false);
        this->declare_parameter("useSideKick", false);
        this->declare_parameter("useLastDirection", false);
        this->declare_parameter("useNearFollowSearchGoal", false);
        this->declare_parameter("firstStateCondition", 0);
        this->declare_parameter("barelang_color", 0);
        this->declare_parameter("dropball", 0);
        this->declare_parameter("team", 0);
    }

    void getParameters()
    {
        ball_panKP = this->get_parameter("ball_panKP").as_double();
        ball_panKD = this->get_parameter("ball_panKD").as_double();
        ball_tiltKP = this->get_parameter("ball_tiltKP").as_double();
        ball_tiltKD = this->get_parameter("ball_tiltKD").as_double();
        frame_X = this->get_parameter("frame_X").as_int();
        frame_Y = this->get_parameter("frame_Y").as_int();
        robotNumber = this->get_parameter("robotNumber").as_int();
        pPanTendang = this->get_parameter("pPanTendang").as_double();
        pTiltTendang = this->get_parameter("pTiltTendang").as_double();
        ballPositioningSpeed = this->get_parameter("ballPositioningSpeed").as_double();
        cSekarang = this->get_parameter("cSekarang").as_double();
        cAktif = this->get_parameter("cAktif").as_double();
        posTiltLocal = this->get_parameter("posTiltLocal").as_double();
        posTiltGoal = this->get_parameter("posTiltGoal").as_double();
        erorrXwalk = this->get_parameter("errorXwalk").as_double();
        erorrYwalk = this->get_parameter("errorYwalk").as_double();
        erorrAwalk = this->get_parameter("errorAwalk").as_double();
        jalan = this->get_parameter("jalan").as_double();
        lari = this->get_parameter("lari").as_double();
        kejar = this->get_parameter("kejar").as_double();
        kejarMid = this->get_parameter("kejarMid").as_double();
        kejarMax = this->get_parameter("kejarMax").as_double();
        tendangJauh = this->get_parameter("tendangJauh").as_int();
        tendangSamping = this->get_parameter("tendangSamping").as_int();
        tendangDekat = this->get_parameter("tendangDekat").as_int();
        sudutTengah = this->get_parameter("sudutTengah").as_int();
        sudutKanan = this->get_parameter("sudutKanan").as_int();
        sudutKiri = this->get_parameter("sudutKiri").as_int();
        rotateGoal_x = this->get_parameter("rotateGoal_x").as_double();
        rotateGoal_y = this->get_parameter("rotateGoal_y").as_double();
        rotateGoal_a = this->get_parameter("rotateGoal_a").as_double();
        myAccrX = this->get_parameter("myAccrX").as_double();
        myAccrY = this->get_parameter("myAccrY").as_double();
        tinggiRobot = this->get_parameter("tinggiRobot").as_int();
        outputSudutY1 = this->get_parameter("outputSudutY1").as_double();
        inputSudutY1 = this->get_parameter("inputSudutY1").as_double();
        outputSudutY2 = this->get_parameter("outputSudutY2").as_double();
        inputSudutY2 = this->get_parameter("inputSudutY2").as_double();
        outputSudutX1 = this->get_parameter("outputSudutX1").as_double();
        inputSudutX1 = this->get_parameter("inputSudutX1").as_double();
        outputSudutX2 = this->get_parameter("outputSudutX2").as_double();
        inputSudutX2 = this->get_parameter("inputSudutX2").as_double();
        useRos = this->get_parameter("useRos").as_bool();
        usePenaltyStrategy = this->get_parameter("usePenaltyStrategy").as_bool();
        useVision = this->get_parameter("useVision").as_bool();
        useImu = this->get_parameter("useImu").as_bool();
        useGameController = this->get_parameter("useGameController").as_bool();
        useCoordination = this->get_parameter("useCoordination").as_bool();
        useLocalization = this->get_parameter("useLocalization").as_bool();
        useFollowSearchGoal = this->get_parameter("useFollowSearchGoal").as_bool();
        useSearchGoal = this->get_parameter("useSearchGoal").as_bool();
        useDribble = this->get_parameter("useDribble").as_bool();
        dribbleOnly = this->get_parameter("dribbleOnly").as_bool();
        useSideKick = this->get_parameter("useSideKick").as_bool();
        useLastDirection = this->get_parameter("useLastDirection").as_bool();
        useNearFollowSearchGoal = this->get_parameter("useNearFollowSearchGoal").as_bool();
        firstStateCondition = this->get_parameter("firstStateCondition").as_int();
        barelang_color = this->get_parameter("barelang_color").as_int();
        team = this->get_parameter("team").as_int();
        dropball = this->get_parameter("dropball").as_int();
    }

    void sendRobotCoordinationData(signed short rNumber, signed short rStatus, signed short sNumber, signed short gridPos, signed short fBall, signed short dBall, signed short gridBall, signed short backIn)
    {
        auto msg = bfc_msgs::msg::Coordination();
        msg.robot_number = rNumber;
        msg.status = rStatus;
        msg.state = sNumber;
        msg.grid_position = gridPos;
        msg.found_ball = fBall;
        msg.distance_ball = dBall;
        msg.grid_ball = gridBall;
        msg.back_in = backIn;
        robotCoordination_->publish(msg);
    }

    void readRobotCoordinationData1(const bfc_msgs::msg::Coordination::SharedPtr message)
    {
        robot1Id = message->robot_number;
        robot1Status = message->status;
        robot1State = message->state;
        robot1GridPosition = message->grid_position;
        robot1FBall = message->found_ball;
        robot1DBall = message->distance_ball;
        robot1GridBall = message->grid_ball;
        robot1BackIn = message->back_in;
    }

    void readRobotCoordinationData2(const bfc_msgs::msg::Coordination::SharedPtr message)
    {
        robot2Id = message->robot_number;
        robot2Status = message->status;
        robot2State = message->state;
        robot2GridPosition = message->grid_position;
        robot2FBall = message->found_ball;
        robot2DBall = message->distance_ball;
        robot2GridBall = message->grid_ball;
        robot2BackIn = message->back_in;
    }

    void readRobotCoordinationData3(const bfc_msgs::msg::Coordination::SharedPtr message)
    {
        robot3Id = message->robot_number;
        robot3Status = message->status;
        robot3State = message->state;
        robot3GridPosition = message->grid_position;
        robot3FBall = message->found_ball;
        robot3DBall = message->distance_ball;
        robot3GridBall = message->grid_ball;
        robot3BackIn = message->back_in;
    }

    void readRobotCoordinationData4(const bfc_msgs::msg::Coordination::SharedPtr message)
    {
        robot4Id = message->robot_number;
        robot4Status = message->status;
        robot4State = message->state;
        robot4GridPosition = message->grid_position;
        robot4FBall = message->found_ball;
        robot4DBall = message->distance_ball;
        robot4GridBall = message->grid_ball;
        robot4BackIn = message->back_in;
    }

    void readRobotCoordinationData5(const bfc_msgs::msg::Coordination::SharedPtr message)
    {
        robot5Id = message->robot_number;
        robot5Status = message->status;
        robot5State = message->state;
        robot5GridPosition = message->grid_position;
        robot5FBall = message->found_ball;
        robot5DBall = message->distance_ball;
        robot5GridBall = message->grid_ball;
        robot5BackIn = message->back_in;
    }

    // Function To Set Timer =====================================================================
    struct timeval t1,
        t2;
    int attends;
    double elapsedTime,
        second;
    bool timer = false;
    void setWaktu()
    {
        elapsedTime =
            second =
                attends = 0;
        timer = false;

        gettimeofday(&t1, NULL);
    }
    // Function For Check Timer
    void cekWaktu(double detik)
    {
        if (attends > 10)
        {
            gettimeofday(&t2, NULL);

            // compute and print the elapsed time in millisec
            elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
            elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;
            second = elapsedTime / 1000.0;
            // printf ("  waktu berlangsung = %.f detik \n\n\n\n", second);

            if (second >= detik)
            {
                timer = true;
            }
            else
            {
                timer = false;
            }
        }
        else
        {
            attends++;
        }
    }

    // Sensor Accelero, Gyro, Angle Value ========================================================
    double accrX, accrY, accrZ,
        gyroX, gyroY, gyroZ,
        angleX, angleY, angleZ,
        myAccrX, myAccrY;
    bool robotJatuh = false,
         robotBergerak = false;
    int countParse = 0,
        countMove = 0;
    int strategyCM, killNrunCM, voltageCM;
    void getSensor()
    {
        RollCM = msg_roll;
        PitchCM = msg_pitch;
        angle = YawCM = msg_yaw;
        strategyNumber = strategyCM = msg_strategy;
        killNrun = msg_kill;

        // printf("battery sekarang = %d\n",voltageCM);

        // menunjukan battery loww
        if (voltageCM < 157)
        {
            lowBatt = true;
            kejarMax = 0.06;
        }
        else
        {
            lowBatt = false;
        }

        // menunjukkan bahwa robot sedang dalam kondisi jatuh -> back to case 0
        // if ((accrX >= -0.5) && (accrX <= 0.9) && (accrY >= -0.9) && (accrY <= 0.9)) { //printf("  robot berdiri...................\n\n");
        if (angleY > 50 || angleY < -50 || angleX > 50 || angleX < -50)
        { // printf("  robot jatuh...................\n\n");
            robotJatuh = true;
        }
        else
        { // printf("  ...................robot berdiri.\n\n");
            robotJatuh = false;
        }

        if (gyroX != 0)
        { // printf("  robot bergerak............\n");
            countMove = 0;
            robotBergerak = true;
        }
        else
        { // printf("  ............robot TDK bergerak\n");
            if (countMove > 10)
            {
                robotBergerak = false;
            }
            else
            {
                countMove++;
            }
        }
    }

    int acclrX;
    bool robotFall = false;
    void getImuSensor()
    {
        acclrX = msg_pitch;
        if (acclrX >= -50 && acclrX <= 50)
        { // printf("  robot berdiri...................\n\n");
            robotFall = false;
        }
        else
        { // printf("  ...................robot jatuh.\n\n");
            robotFall = true;
        }
    }

    // Sudut 20 Servo ===========================================================================
    double head1, head2,
        Larm1, Larm2, Larm3,
        Lleg1, Lleg2, Lleg3, Lleg4, Lleg5, Lleg6,
        Rleg1, Rleg2, Rleg3, Rleg4, Rleg5, Rleg6,
        Rarm1, Rarm2, Rarm3;
    int countParses = 0;
    void getServPos()
    {
        char line[200];
        char *sprs;

        FILE *outputfp;
        outputfp = fopen("../Player/PosNote", "r");
        fscanf(outputfp, "%s", &line);
        countParses = 0;
        sprs = strtok(line, ";");
        while (sprs != NULL)
        {
            if (countParses == 0)
            {
                sscanf(sprs, "%lf", &head1);
            } // ID=19
            else if (countParses == 1)
            {
                sscanf(sprs, "%lf", &head2);
            } // ID=20
            else if (countParses == 2)
            {
                sscanf(sprs, "%lf", &Larm1);
            } // ID=2
            else if (countParses == 3)
            {
                sscanf(sprs, "%lf", &Larm2);
            } // ID=4
            else if (countParses == 4)
            {
                sscanf(sprs, "%lf", &Larm3);
            } // ID=6
            else if (countParses == 5)
            {
                sscanf(sprs, "%lf", &Lleg1);
            } // ID=8
            else if (countParses == 6)
            {
                sscanf(sprs, "%lf", &Lleg2);
            } // ID=10
            else if (countParses == 7)
            {
            } // ID=12
            else if (countParses == 8)
            {
                sscanf(sprs, "%lf", &Lleg4);
            } // ID=14
            else if (countParses == 9)
            {
                sscanf(sprs, "%lf", &Lleg5);
            } // ID=16
            else if (countParses == 10)
            {
                sscanf(sprs, "%lf", &Lleg6);
            } // ID=18
            else if (countParses == 11)
            {
                sscanf(sprs, "%lf", &Rleg1);
            } // ID=7
            else if (countParses == 12)
            {
                sscanf(sprs, "%lf", &Rleg2);
            } // ID=9
            else if (countParses == 13)
            {
                sscanf(sprs, "%lf", &Rleg3);
            } // ID=11
            else if (countParses == 14)
            {
                sscanf(sprs, "%lf", &Rleg4);
            } // ID=13
            else if (countParses == 15)
            {
                sscanf(sprs, "%lf", &Rleg5);
            } // ID=15
            else if (countParses == 16)
            {
                sscanf(sprs, "%lf", &Rleg6);
            } // ID=17
            else if (countParses == 17)
            {
                sscanf(sprs, "%lf", &Rarm1);
            } // ID=1
            else if (countParses == 18)
            {
                sscanf(sprs, "%lf", &Rarm2);
            } // ID=3
            else if (countParses == 19)
            {
                sscanf(sprs, "%lf", &Rarm3);
                countParses = -1;
            } // ID=5
            sprs = strtok(NULL, ";");
            countParses++;
        }
        fclose(outputfp);
        // printf("  head(%.lf, %.lf)", head1, head2);
        // printf("  Larm(%.lf, %.lf, %.lf)", Larm1, Larm2, Larm3);
        // printf("  Rarm(%.lf, %.lf, %.lf)", Rarm1, Rarm2, Rarm3);
        // printf("  Lleg(%.lf, %.lf, %.lf, %.lf, %.lf, %.lf)", Lleg1, Lleg2, Lleg3, Lleg4, Lleg5, Lleg6);
        // printf("  Rleg(%.lf, %.lf, %.lf, %.lf, %.lf, %.lf)", Rleg1, Rleg2, Rleg3, Rleg4, Rleg5, Rleg6);
    }

    // Checking Lost Ball ========================================================================
    int countBallLost = 0,
        countBallFound = 0,
        returnBallVal;
    int ballLost(int threshold)
    {
        if (useVision)
        {
            if (Ball_X == -1 && Ball_Y == -1)
            {
                countBallFound = 0;
                countBallLost++;
                if (countBallLost >= threshold)
                {
                    returnBallVal = 1;
                }
            }
            else
            {
                countBallLost = 0;
                countBallFound++;
                if (countBallFound > 1)
                {
                    returnBallVal = 0;
                }
            }
        }
        else
        {
            countBallFound = 0;
            countBallLost++;
            if (countBallLost >= threshold)
            {
                returnBallVal = 1;
            }
        }
        return returnBallVal;
    }

    // Search ball ===============================================================================
    double tiltRate = -0.05,
           panRate = -0.05,
           searchKe = 0,

           batasKanan = -1.6,
           batasKiri = 1.6,
           batasAtas = -2.0,
           batasBawah = -0.6;

    void tiltSearchBall(double tempPosPan)
    { // printf("  tiltSearchBall\n\n");
        posPan = tempPosPan;
        posTilt += tiltRate;

        if (posTilt <= batasAtas || posTilt >= batasBawah)
        {
            tiltRate *= -1;
        }

        if (posTilt <= batasAtas)
        {
            posTilt = batasAtas;
        }
        else if (posTilt >= batasBawah)
        {
            posTilt = batasBawah;
        }

        headMove(posPan, posTilt); // printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
    }

    void panSearchBall(double tempPosTilt)
    { // printf("  panSearchBall\n\n");
        posTilt = tempPosTilt;
        posPan += panRate;

        if (posPan <= batasKanan || posPan >= batasKiri)
        {
            panRate *= -1;
        }

        if (headPan <= (0.0 + panRate) && headPan >= (0.0 - panRate))
        {
            // searchKe += 0.5;
        }

        if (posPan >= batasKiri)
        {
            posPan = batasKiri;
        }
        else if (posPan <= batasKanan)
        {
            posPan = batasKanan;
        }

        headMove(posPan, posTilt); // printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
    }

    int i = 1,
        panKe = 2,
        tiltKe = 1;
    double panSearch[5] = {1.6, 0.8, 0.0, -0.8, -1.6},
           // tiltSearch1[3] = {-0.6, -1.2, -1.8},
        tiltSearch1[3] = {-0.8, -1.4, -1.8},
           tiltSearch2[2] = {-0.8, -1.4};
    void SearchBall(int mode)
    { // printf("  normalSearchBall\n\n");
        if (mode == 1)
        { // (atas-bawah)
            posTilt += tiltRate;
            if (posTilt <= batasAtas || posTilt >= batasBawah)
            {
                if (panKe == 2)
                {
                    searchKe += 1;
                }

                tiltRate *= -1;
                panKe += i;
                if (panKe >= 4 || panKe <= 0)
                {
                    i = -i;
                }
            }
            posPan = panSearch[panKe];
            printf("count pan = %d\n", panKe);
        }
        else if (mode == 2)
        { // (kiri-kanan)
            posPan += panRate;
            if (posPan <= batasKanan || posPan >= batasKiri)
            {
                if (tiltKe == 1)
                {
                    searchKe += 1;
                }
                panRate *= -1;
                tiltKe += i;

                if (tiltKe >= 2 || tiltKe <= 0)
                {
                    i = -i;
                }
            }
            posTilt = tiltSearch1[tiltKe]; // printf("count tilt = %d\n", tiltKe);
        }
        else if (mode == 3)
        { // muter-muter
            posPan += panRate;
            if (posPan <= batasKanan || posPan >= batasKiri)
            {
                panRate *= -1;
                countTilt++;
                if (countTilt > 1)
                    countTilt = 0;
            }
            posTilt = tiltSearch2[countTilt]; // printf("count tilt = %d\n", countTilt);
        }

        if (posPan >= batasKiri)
        {
            posPan = batasKiri;
        }
        else if (posPan <= batasKanan)
        {
            posPan = batasKanan;
        }
        if (posTilt <= batasAtas)
        {
            posTilt = batasAtas;
        }
        else if (posTilt >= batasBawah)
        {
            posTilt = batasBawah;
        }

        headMove(posPan, posTilt); // printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
    }

    int sabar = 0,
        tiltPos = 0;
    void threeSearchBall()
    {
        if (sabar > 7)
        {
            posPan += panRate;
            if (posPan <= batasKanan || posPan >= batasKiri)
            {
                if (tiltPos == 2 && posPan <= -1.5)
                {
                    searchKe += 1;
                }
                panRate *= -1;
                tiltPos += i;
                if (tiltPos >= 2 || tiltPos <= 0)
                {
                    i = -i;
                }
            }
            posTilt = tiltSearch1[tiltPos]; // printf("count tilt = %d\n", tiltPos);
        }
        else
        {
            posPan = 1.45;
            posTilt = -0.8;
            tiltPos = 0;
            searchKe = 0;
            i = 1;
            tiltRate = -0.05;
            panRate = -0.05;
            sabar++;
        }

        if (posPan >= batasKiri)
        {
            posPan = batasKiri;
        }
        else if (posPan <= batasKanan)
        {
            posPan = batasKanan;
        }
        if (posTilt <= batasAtas)
        {
            posTilt = batasAtas;
        }
        else if (posTilt >= batasBawah)
        {
            posTilt = batasBawah;
        }

        headMove(posPan, posTilt); // printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
    }

    void rightSearchBall()
    { // printf("  rightSearchBall\n\n");
        posTilt += tiltRate;
        if (posTilt <= batasAtas || posTilt >= batasBawah)
        {
            tiltRate *= -1;
            posPan += panRate;
            if (posPan >= 0.0 || posPan <= batasKanan)
            {
                panRate *= -1;
            }
        }

        if (posPan >= 0.0)
        {
            posPan = 0.0;
        }
        else if (posPan <= batasKanan)
        {
            posPan = batasKanan;
        }
        if (posTilt <= batasAtas)
        {
            posTilt = batasAtas;
        }
        else if (posTilt >= batasBawah)
        {
            posTilt = batasBawah;
        }

        headMove(posPan, posTilt);
    }

    void leftSearchBall()
    { // printf("  leftSearchBall\n\n");
        posTilt += tiltRate;
        if (posTilt <= batasAtas || posTilt >= batasBawah)
        {
            tiltRate *= -1;
            posPan += panRate;
            if (posPan >= batasKiri || posPan <= 0.0)
            {
                panRate *= -1;
            }
        }

        if (posPan <= 0.0)
        {
            posPan = 0.0;
        }
        else if (posPan >= batasKiri)
        {
            posPan = batasKiri;
        }
        if (posTilt <= batasAtas)
        {
            posTilt = batasAtas;
        }
        else if (posTilt >= batasBawah)
        {
            posTilt = batasBawah;
        }

        headMove(posPan, posTilt);
    }

    double invPan;
    void searchBallPan(double uPan, double uTilt)
    {
        posTilt = uTilt;
        invPan = uPan * -1;

        posPan += panRate;

        if (posPan <= invPan)
        {
            posPan = invPan;
            panRate *= -1;
        }
        else if (posPan >= uPan)
        {
            posPan = uPan;
            panRate *= -1;
        }

        headMove(posPan, posTilt);
    }

    bool neckX;
    void searchBallRectang(double atas, double kanan, double bawah, double kiri)
    {
        if (neckX)
        {
            posPan += panRate;
            if (posPan >= kiri || posPan <= kanan)
            {
                panRate *= -1;
                neckX = false;
            }
        }
        else
        {
            posTilt += tiltRate;
            if (posTilt <= atas || posTilt >= bawah)
            {
                tiltRate *= -1;
                neckX = true;
            }
        }

        if (posPan >= kiri)
        {
            posPan = kiri;
        }
        else if (posPan <= kanan)
        {
            posPan = kanan;
        }
        if (posTilt <= atas)
        {
            posTilt = atas;
        }
        else if (posTilt >= bawah)
        {
            posTilt = bawah;
        }

        headMove(posPan, posTilt); // printf("pan = %f, tilt = %f\n",posPan,posTilt);
    }

    double ballPan = 0,
           ballTilt = 0;
    void saveBallLocation()
    {
        //	trackBall();
        ballPan = posPan;
        ballTilt = posTilt;
    }

    void loadBallLocation(double tilt)
    {
        // posPan = 0.0;
        // posTilt = -0.8;
        posPan = ballPan;
        posTilt = ballTilt + tilt;
        headMove(posPan, posTilt);
    }

    int panDegree, tiltRad = 0;
    double panRad = 0;
    void savePan()
    {
        panRad = headPan;
        tiltRad = posTilt;
        panDegree = (posPan * (180 / PI)) + angle;
        panRad = panRad * -1;
        panDegree = panDegree * -1;
    }

    double koorRobotX,
        koorRobotY = 0;
    void saveKoordinatRobot()
    {
        koorRobotX = robotPos_X;
        koorRobotY = robotPos_Y;
    }

    void loadKoordinatRobot()
    {
        // resetOdometry();
        // robotPos_X = koorRobotX;
        // robotPos_Y = koorRobotY;
        initialPos_X = koorRobotX;
        initialPos_Y = koorRobotY;
        deltaPos_X = deltaPos_Y = 0;
    }

    int semeh;
    int koordinasiJarak()
    {
        semeh = (int)(headTilt * -100);
        return semeh;
    }

    // Ball Tracking =============================================================================
    double intPanB = 0, dervPanB = 0, errorPanB = 0, preErrPanB = 0,
           PPanB = 0, IPanB = 0, DPanB = 0,
           intTiltB = 0, dervTiltB = 0, errorTiltB = 0, preErrTiltB = 0,
           PTiltB = 0, ITiltB = 0, DTiltB = 0,
           dtB = 0.04;
    double B_Pan_err_diff, B_Pan_err, B_Tilt_err_diff, B_Tilt_err, B_PanAngle, B_TiltAngle,
        pOffsetB, iOffsetB, dOffsetB,
        errorPanBRad, errorTiltBRad,
        offsetSetPointBall;
    void trackBall()
    {
        if (useVision)
        {
            if (Ball_X != -1 && Ball_Y != -1)
            { // printf("Tracking");
                ballPos_X = BallCoor_X;
                ballPos_Y = BallCoor_Y;
                // mode 1 ######################################################################
                /*// PID pan ==========================================================
                errorPanB  = (double)Ball_x - (frame_X / 2);//160
                PPanB  = errorPanB  * 0.00010; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya

                intPanB += errorPanB * dtB;
                IPanB = intPanB * 0.0;

                dervPanB = (errorPanB - preErrPanB) / dtB;
                DPanB = dervPanB * 0.00001;

                preErrPanB = errorPanB;

                //posPan += PPanB*-1; //dikali -1 kalau receler terbalik dalam pemasangan
                posPan += (PPanB + IPanB + DPanB) * -1;


                // PID tilt ==========================================================
                errorTiltB = (double)Ball_y - (frame_Y / 2);//120
                PTiltB = errorTiltB * 0.00010; // Tune in Kp Tilt 0.00030

                intTiltB += errorTiltB * dtB;
                ITiltB = intTiltB * 0.0;

                dervTiltB = (errorTiltB - preErrTiltB) / dtB;
                DTiltB = dervTiltB * 0.00001;

                preErrTiltB = errorTiltB;

                //posTilt += PTiltB;
                posTilt += (PTiltB + ITiltB + DTiltB);*/

                // mode 2 ######################################################################
                //  offsetSetPointBall = ((int)(posTilt * 30)+7); //+54
                //  if (offsetSetPointBall > 36) offsetSetPointBall = 36;
                //  else if (offsetSetPointBall < 0) offsetSetPointBall = 0;

                // errorPanB  = (double)Ball_X - ((frame_X / 2) + offsetSetPointBall);//160
                errorPanB = (double)Ball_X - (frame_X / 2);
                errorTiltB = (double)Ball_Y - (frame_Y / 2); // 120
                errorPanB *= -1;
                errorTiltB *= -1;
                errorPanB *= (90 / (double)frame_X);  // pixel per angle
                errorTiltB *= (60 / (double)frame_Y); // pixel per angle
                // errorPanB *= (77.32 / (double)frame_X); // pixel per angle
                // errorTiltB *= (61.93 / (double)frame_Y); // pixel per angle

                errorPanBRad = (errorPanB * PI) / 180;
                errorTiltBRad = (errorTiltB * PI) / 180;
                // printf("errorPan = %.2f \t errorTilt = %.2f\n", errorPanB, errorTiltB);
                // printf("RadrrorPan = %.2f \t RaderrorTilt = %.2f\n", errorPanBRad, errorTiltBRad);
                // printf("KPPan = %f \t KDPan = %f\t", kamera.panKP, kamera.panKD); printf("KPTilt = %f \t KDTilt = %f\n", kamera.tiltKP, kamera.tiltKD);

                B_Pan_err_diff = errorPanBRad - B_Pan_err;
                B_Tilt_err_diff = errorTiltBRad - B_Tilt_err;

                // PID pan ==========================================================
                // PPanB  = B_Pan_err  * kamera.panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
                PPanB = B_Pan_err * ball_panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
                intPanB += B_Pan_err * dtB;
                IPanB = intPanB * 0.0;
                dervPanB = B_Pan_err_diff / dtB;
                // DPanB = dervPanB * kamera.panKD;
                DPanB = dervPanB * ball_panKD;
                B_Pan_err = errorPanBRad;
                posPan += (PPanB + IPanB + DPanB);

                // PID tilt ==========================================================
                // PTiltB = B_Tilt_err * kamera.tiltKP; // Tune in Kp Tilt 0.00030
                PTiltB = B_Tilt_err * ball_tiltKP; // Tune in Kp Tilt 0.00030

                intTiltB += B_Tilt_err * dtB;
                ITiltB = intTiltB * 0.0;

                dervTiltB = B_Tilt_err_diff / dtB;
                // DTiltB = dervTiltB * kamera.tiltKD;
                DTiltB = dervTiltB * ball_tiltKD;

                preErrTiltB = errorTiltB;
                B_Tilt_err = errorTiltBRad;
                posTilt += (PTiltB + ITiltB + DTiltB) * -1;

                if (posPan >= 1.6)
                {
                    posPan = 1.6;
                }
                else if (posPan <= -1.6)
                {
                    posPan = -1.6;
                }
                if (posTilt <= -2.0)
                {
                    posTilt = -2.0;
                }
                else if (posTilt >= -0.4)
                {
                    posTilt = -0.4;
                }

                headMove(posPan, posTilt); // printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);

                koordinasiJarak();
                saveBallLocation();
            }
        }
    }

    // Body Tracking Ball ========================================================================
    double errorBodyPosition,
        bodyP_Controller;
    int bodyTrue = 0,
        delayTrue = 0;
    int bodyTrackingBall(int threshold)
    {
        errorBodyPosition = 0 - headPan;
        bodyP_Controller = errorBodyPosition * -0.5; //-0.5

        if (ballLost(20))
        {
            Walk(0.0, 0.0, 0.0);
            bodyP_Controller = bodyTrue = delayTrue = 0;
        }
        else
        {
            if (errorBodyPosition >= -0.3 && errorBodyPosition <= 0.3)
            { // untuk hasil hadap 0.8
                // motion("0");
                Walk(0.0, 0.0, 0.0);
                delayTrue++;
            }
            else
            {
                trackBall();
                // motion("9");

                if (bodyP_Controller < 0)
                {
                    bodyP_Controller = -0.2;
                } // kanan 0.15
                else
                {
                    bodyP_Controller = 0.2;
                } // kiri 0.15

                bodyTrue = delayTrue = 0;
                Walk(0.0, 0.0, bodyP_Controller);
            }

            if (delayTrue >= threshold)
            {
                bodyTrue = 1;
            }
            else
            {
                bodyTrue = 0;
            }
        } // printf("Body Error = %.2f\t Body P Controller = %.2f\n",errorBodyPosition,bodyP_Controller);
        return bodyTrue;
    }

    // Hitung Jarak Bola berdasarkan headTilt ==============================================================
    double alphaY, // hasil derajat ketika headTilt
        betaY,
        inputY, // nilai realtime headTilt

        alphaX, // hasil derajat ketika headPan
        betaX,
        inputX, // nilai realtime headPan

        jarakBola_Y, // hasil jarak(cm) dari kalkulasi headTilt
        jarakBola_X, // hasil jarak(cm) dari kalkulasi headPan
        jarakBola;

    void kalkulasiJarakBola()
    {
        inputY = headTilt;
        inputX = headPan;

        // alphaY = -57.29 * headTilt; //(metode 1)
        alphaY = outputSudutY1 + ((outputSudutY2 - outputSudutY1) / (inputSudutY2 - inputSudutY1)) * (inputY - inputSudutY1); //(metode 2) //printf("  alphaY = %.2f,", alphaY);
        betaY = 180 - (90 + alphaY);                                                                                          // printf("  betaY = %.2f,", betaY);

        // alphaX = -57.29 * headPan; //(metode 1)
        alphaX = outputSudutX1 + ((outputSudutX2 - outputSudutX1) / (inputSudutX2 - inputSudutX1)) * (inputX - inputSudutX1); //(metode 2) //printf("  alphaX = %.2f,", alphaX);
        betaX = 180 - (90 + alphaX);                                                                                          // printf("  betaX = %.2f,", betaX);

        // sin & cos dalam c++ adalah radian, oleh karena itu harus : sin( ... * PI / 180)
        jarakBola_Y = (tinggiRobot * sin(alphaY * PI / 180)) / sin(betaY * PI / 180); // printf("  jarakBola_Y = %.f,", jarakBola_Y);
        jarakBola_X = (jarakBola_Y * sin(alphaX * PI / 180)) / sin(betaX * PI / 180); // printf("  jarakBola_X = %.f\n\n\n\n", jarakBola_X);
        jarakBola = sqrt((jarakBola_Y * jarakBola_Y) + (jarakBola_X * jarakBola_X));

        // regresi (metode 3)
        // jarakBola_Y = (-933.9*(pow(posTilt,5))) + (-5340.8*(pow(posTilt,4))) + (-12018*(pow(posTilt,3))) + (-13183*(pow(posTilt,2))) + (-7050.2*posTilt) - 1454.3;
    }

    // Untuk Kalkulasi Posisi P1
    double P1_X, P1_Y;
    void hitungKoordinatBolaP1()
    {
        // mode1--------------
        // kalkulasiJarakBola();
        // P1_X = jarakBola_X;
        // P1_Y = jarakBola_Y;
        // mode2--------------
        P1_X = Ball_Y;
        P1_Y = Ball_X;
        // printf("  P1_X = %.2f,  P1_Y = %.2f,", P1_X, P1_Y);
    }

    // Untuk Kalkulasi Posisi P2
    double P2_X, P2_Y;
    void hitungKoordinatBolaP2()
    {
        // mode1--------------
        // kalkulasiJarakBola();
        // P2_X = jarakBola_X;
        // P2_Y = jarakBola_Y;
        // mode2--------------
        P2_X = Ball_Y;
        P2_Y = Ball_X;
        // printf("  P2_X = %.2f,  P2_Y = %.2f,", P2_X, P2_Y);
    }

    // Untuk penyebut dan pembilang gradient
    double deltaY, deltaX;
    void hitungDeltaY()
    {
        deltaY = P2_Y - P1_Y; // mendekat positif
        // deltaY = P1_Y - P2_Y; //mendekat positif
        // printf("  deltaY = %.2f,", deltaY);
    }

    void hitungDeltaX()
    {
        deltaX = P2_X - P1_X; // mendekat negatif
        // deltaX = P1_X - P2_X; //mendekat positif
        // printf("  deltaX = %.2f,\n\n", deltaX);
    }

    // Untuk Gradient
    double gradient;
    void hitungGradient()
    {
        gradient = deltaY / deltaX;
        // printf("  gradient = %.2f,", gradient);
    }

    int cntOke1,
        cntOke2,
        cntUlang,
        kondisiBola = 0;
    bool oke = true,
         ulang = false;
    void hitungGerakBola()
    { // Bagian pengecekan pergerakan bola
        if (Ball_X == -1 && Ball_Y == -1)
        { // bola hilang
            deltaY =
                deltaX =
                    jarakBola_X =
                        jarakBola_Y = 0;
        }

        if (ulang)
        { // printf("ulang \n");
            cntOke1 =
                cntOke2 = 0;
            if (cntUlang > 5)
            {
                ulang = false;
                oke = true;
            }
            else
            {
                deltaY =
                    deltaX =
                        jarakBola_X =
                            jarakBola_Y = 0;

                cntUlang++;
            }
        }

        if (oke)
        { // printf("oke \n");
            cntUlang = 0;
            if (cntOke1 > 5)
            {
                hitungKoordinatBolaP2();

                hitungDeltaY();
                hitungDeltaX();
                // hitungGradient();

                if (cntOke2 > 10)
                {
                    oke = false;
                    ulang = true;
                }
                else
                {
                    cntOke2++;
                }
            }
            else
            {
                hitungKoordinatBolaP1();
                cntOke1++;
            }
        }

        // if ((deltaY >= 20 && deltaX <= -20) || (deltaY >= 20 && deltaX >= 20)) { //0.5 //7.0
        if (deltaY >= 0.5)
        { // 0.5 //7.0 // 30
            // printf("  deltaY = %.f,", deltaY);
            // printf("  Bola Menjauh\n");
            kondisiBola = 1;
            //} else if((deltaY <= -20 && deltaX <= -20) || (deltaY <= -20 && deltaX >= 20)) { //-2 //-1.4
        }
        else if (deltaY <= -0.5)
        { //-2 //-1.4 // -30
            // printf("  deltaY = %.f,", deltaY);
            // printf("  Bola Mendekat\n");
            kondisiBola = -1;
        }
        else if (deltaY >= -0.3 && deltaY <= 0.3 && deltaX >= -0.3 && deltaX <= 0.3)
        { //-2 //-1.4 // -5
            // printf("  Bola Diam");
            kondisiBola = 0;
        }
    }

    // Follow Ball ===============================================================================
    int countReadyKick;
    double SetPointPan = 0,
           SetPointTilt = -0.8, //-0.08
        errorfPan,
           errorfTilt,
           PyMove = 0,
           PxMove = 0,
           PaMove = 0;
    void followBall(int mode)
    { // 0 normal, 1 sambil belok
        trackBall();

        if (posTilt >= SetPointTilt)
        {
            posTilt = SetPointTilt;
        }
        else if (posTilt < -2.0)
        {
            posTilt = -2.0;
        }

        errorfPan = posPan - SetPointPan;
        errorfTilt = posTilt - SetPointTilt;

        if (posTilt >= SetPointTilt && posPan < 0.4 && posPan > -0.4 && Ball_X != -1 && Ball_Y != -1)
        { // Stop(bola sudah dekat)
            countReadyKick++;
        }
        else
        { // Kejar Bola(bola masih jauh)
            countReadyKick = 0;
        }

        if (countReadyKick >= 1)
        {                               // 5
            PxMove = 0.0;               // jalan ditempat
            PyMove = errorfPan * 0.040; // 0.045
            PaMove = errorfPan * 0.20;  // 0.30; //0.045
        }
        else
        {
            if (headTilt < -1.3)
            {
                PxMove = kejarMax; // 0.08
            }
            else if (headTilt >= -1.3 && headTilt < -1.2)
            {
                PxMove = kejarMid; // 0.07
            }
            else if (headTilt > -1.0)
            {
                PxMove = lari; // 0.05
            }
            else
            {
                PxMove = kejar; // 0.06
            }
            // PxMove = errorfTilt * 0.1 * -13; //Robot besar 0.13, robot kecil 0.1
            // PxMove = (0.08 / (-1.6)) * (posTilt); //0.04-0.06
            PyMove = errorfPan * 0.40; // 0.125; //0.045
            PaMove = errorfPan * 0.30; // 0.25; //0.35; //0.045
        }

        if (mode == 0)
        { // Mode differential walking
            if (errorfPan > -0.4 && errorfPan < 0.4)
            { // printf("AAAAAAAA\n");
                Walk(PxMove, 0.0, PaMove);
            }
            else
            { // printf("BBBBBBBB\n");
                Walk(0.0, 0.0, PaMove);
            }
        }
        else if (mode == 1)
        { // Mode omnidirectional walking
            if (errorfPan > -0.4 && errorfPan < 0.4)
            { // printf("CCCCCCCC\n");
                Walk(PxMove, PyMove, PaMove);
            }
            else
            { // printf("DDDDDDDD\n");
                Walk(0.0, 0.0, PaMove);
            }
        }
    }

    // Get Imu Value =================================================================
    void sudut()
    {
        angle = YawCM;

        RobotAngle = YawCM;
        if (angle > 0)
        {
            sendAngle = angle;
        }
        else if (angle < 0)
        {
            sendAngle = angle + 360;
        }
        else if (angle == 0)
        {
            sendAngle = 0;
        }

        if (angle > 180)
        {
            angle = 180;
        }
        else if (angle < -180)
        {
            angle = -180;
        }
        // printf("  Sudut = %.f", angle);
    }

    // IMU ===========================================================================
    int setPoint1,
        setPoint2;

    double errorCPosPan,
        errorCPosTilt,
        alfaImu,
        bodyYImu,
        bodyXImu;

    //	X+ = maju
    //	X- = mundur
    //	Y+ = samping kiri
    //	Y- = samping kanan
    //	A+ = putar kiri
    //	A- = putar kanan

    void rotateDirec(int arah, double jarakTilt)
    {
        errorCPosPan = posPan;                 // adalah nilai tengah pan, dan menjadi titik berhenti jika telah tepenuhi
        errorCPosTilt = posTilt - (jarakTilt); //-0.45 adalah nilai tengah tilt, robot akan jalan ditempat(tidak maju/mundur) jika nilai terpenuhi

        bodyXImu = errorCPosTilt * (-0.1);          // nilai pengali ini harus tetap bernilai negatif //besarnya kalkulasi maju/mundur yg dibutuhkan tehadap posTilt
        bodyYImu = abs(errorCPosPan / 100) + 0.018; // 0.017;
        alfaImu = errorCPosPan * 0.7;               // 0.7; nilai pengali ini harus tetap bernilai positif //besarnya kalkulasi rotate yg dibutuhkan tehadap posPan

        if (arah <= 0)
        { // rotate ke kanan
            // alfaImu = -0.15; bodyYImu = 0.017;
            if (bodyYImu < 0)
            {
                bodyYImu = -bodyYImu;
            }
        }
        else
        { // rotate ke kiri
            // alfaImu = 0.15; bodyYImu = -0.017;
            if (bodyYImu > 0)
            {
                bodyYImu = -bodyYImu;
            }
        }
    }

    void rotateParabolic(int arah, double jarakTilt)
    {
        rotateDirec(arah, jarakTilt);
        Walk(bodyXImu, bodyYImu, alfaImu);
    }

    bool robotDirection = false;
    int eleh = 0,
        btsRotate = 0,
        btsSetPoint = 0;
    void Imu(int gawang, double jarakTilt)
    { // opsi 2, menggunakan "mode" untuk mengecek robot direction
        trackBall();

        if (gawang > 180)
        {
            gawang = 180;
        }
        else if (gawang < -180)
        {
            gawang = -180;
        }

        sudut();

        setPoint1 = 10 + gawang;  // 10 //15 //20 //60
        setPoint2 = -10 + gawang; // 10 //15 //20 //60

        if (setPoint1 > 180 || setPoint2 < -180)
        { // jika arah imu dibelakang
            if (setPoint1 > 180)
            { // nilai setpoint1 diubah jd negatif
                btsSetPoint = setPoint1 - 360;

                if (angle >= setPoint2 || angle <= btsSetPoint)
                {
                    bodyXImu = bodyYImu = alfaImu = 0.0;
                    if (eleh > 10)
                    {
                        robotDirection = true;
                    }
                    else
                    {
                        eleh++;
                    }
                }
                else
                {
                    if (gawang >= 0)
                    {
                        btsRotate = gawang - 180;
                        if ((angle <= gawang) && (angle >= btsRotate))
                        {
                            rotateDirec(-1, jarakTilt); // printf("  rotate ke kanan\n\n");
                        }
                        else
                        {
                            rotateDirec(1, jarakTilt); // printf("  rotate ke kiri\n\n");
                        }
                    }
                    else
                    {
                        btsRotate = gawang + 180;
                        if ((angle >= gawang) && (angle <= btsRotate))
                        {
                            rotateDirec(1, jarakTilt); // printf("  rotate ke kiri\n\n");
                        }
                        else
                        {
                            rotateDirec(-1, jarakTilt); // printf("  rotate ke kanan\n\n");
                        }
                    }

                    eleh = 0;
                    robotDirection = false;
                }
                Walk(bodyXImu, bodyYImu, alfaImu);
            }
            else
            { // nilai setPoint2 diubah jadi positif
                btsSetPoint = setPoint2 + 360;

                if (angle >= btsSetPoint || angle <= setPoint1)
                {
                    bodyXImu = bodyYImu = alfaImu = 0.0;
                    if (eleh > 10)
                    {
                        robotDirection = true;
                    }
                    else
                    {
                        eleh++;
                    }
                }
                else
                {
                    if (gawang >= 0)
                    {
                        btsRotate = gawang - 180;
                        if ((angle <= gawang) && (angle >= btsRotate))
                        {
                            rotateDirec(-1, jarakTilt); // printf("  rotate ke kanan\n\n");
                        }
                        else
                        {
                            rotateDirec(1, jarakTilt); // printf("  rotate ke kiri\n\n");
                        }
                    }
                    else
                    {
                        btsRotate = gawang + 180;
                        if ((angle >= gawang) && (angle <= btsRotate))
                        {
                            rotateDirec(1, jarakTilt); // printf("  rotate ke kiri\n\n");
                        }
                        else
                        {
                            rotateDirec(-1, jarakTilt); // printf("  rotate ke kanan\n\n");
                        }
                    }

                    eleh = 0;
                    robotDirection = false;
                }
                Walk(bodyXImu, bodyYImu, alfaImu);
            }
        }
        else
        { // arah imu kedepan
            if (angle >= setPoint2 && angle <= setPoint1)
            {
                bodyXImu = bodyYImu = alfaImu = 0.0;
                if (eleh > 10)
                {
                    robotDirection = true;
                }
                else
                {
                    eleh++;
                }
            }
            else
            {
                if (gawang >= 0)
                {
                    btsRotate = gawang - 180;
                    if ((angle <= gawang) && (angle >= btsRotate))
                    {
                        rotateDirec(-1, jarakTilt); // printf("  rotate ke kanan\n\n");
                    }
                    else
                    {
                        rotateDirec(1, jarakTilt); // printf("  rotate ke kiri\n\n");
                    }
                }
                else
                {
                    btsRotate = gawang + 180;
                    if ((angle >= gawang) && (angle <= btsRotate))
                    {
                        rotateDirec(1, jarakTilt); // printf("  rotate ke kiri\n\n");
                    }
                    else
                    {
                        rotateDirec(-1, jarakTilt); // printf("  rotate ke kanan\n\n");
                    }
                }

                eleh = 0;
                robotDirection = false;
            }
            Walk(bodyXImu, bodyYImu, alfaImu);
        }
    }

    // Grid Coordinate ================================================================
    double outGrid;
    void gridCoor()
    {
        if (Grid >= 1 && Grid <= 12)
        {
            outGrid = 0;
        }
        else if (Grid >= 13 && Grid <= 42)
        {
            outGrid = ArchSinEnemy;
        }
        else if (Grid == 43)
        {
            outGrid = 70;
        }
        else if (Grid == 44)
        {
            outGrid = 45;
        }
        else if (Grid == 45)
        {
            outGrid = 25;
        }
        else if (Grid == 46)
        {
            outGrid = -25;
        }
        else if (Grid == 47)
        {
            outGrid = -45;
        }
        else if (Grid == 48)
        {
            outGrid = -70;
        }
        else if (Grid == 49)
        {
            outGrid = 75;
        }
        else if (Grid == 50)
        {
            outGrid = 60;
        }
        else if (Grid == 51)
        {
            outGrid = 20;
        }
        else if (Grid == 52)
        {
            outGrid = -20;
        }
        else if (Grid == 53)
        {
            outGrid = -60;
        }
        else if (Grid == 54)
        {
            outGrid = -75;
        }

        // untuk kondisi followSearchGoal
        if (Activated)
        {
            if (Grid >= 43 && Grid <= 54)
                followSearchAktif = true;
            else
                followSearchAktif = false;
        }
    }

    // Rotate Body with IMU ===========================================================
    bool posRotate = false;

    void rotateBodyImu(int rotate)
    {
        trackBall();

        sudut();

        setPoint1 = 5 + rotate;  // 20 bisa kemungkinan 180 keatas
        setPoint2 = -5 + rotate; // 20 bisa kemungkinan -180 bawah

        if (angle > setPoint2 && angle < setPoint1)
        {
            posRotate = true;
        }
        else
        {
            if (angle > rotate)
            { // putar Kiri
                Walk(0.0, 0.0, 0.2);
            }
            else
            { // putar Kanan
                Walk(0.0, 0.0, -0.2);
            }
            // posRotate = false;
        }
    }

    // Rotate Body with IMU ===========================================================
    bool posRotateNew = false;
    double IaMove, errorImuNew,
        putar;
    void rotateBodyImuNew(int rotate)
    {
        trackBall();

        sudut();

        setPoint1 = 5 + rotate;  // 10 //15 //20 //60
        setPoint2 = -5 + rotate; // 10 //15 //20 //60

        if (setPoint1 > 180 || setPoint2 < -180)
        { // jika arah imu dibelakang
            if (setPoint1 > 180)
            { // nilai setpoint1 diubah jd negatif
                btsSetPoint = setPoint1 - 360;

                if (angle >= setPoint2 || angle <= btsSetPoint)
                { // misal 170 ke -170
                    PaMove = 0.00;
                    posRotateNew = true;
                }
                else
                {
                    btsRotate = rotate - 180;
                    if ((angle <= rotate) && (angle >= btsRotate))
                    {                                      // misal di range 0 - 180, maka putar kanan
                        putar = (rotate - angle) * 0.0065; // 0.0033
                        PaMove = -putar;
                    }
                    else
                    { // putar kiri
                        if (angle > rotate)
                        {
                            putar = (angle - rotate) * 0.0065;
                        } // 0.0033
                        else
                        {
                            putar = ((180 - rotate) + (180 + angle)) * 0.0065;
                        } // 0.0033
                        PaMove = putar;
                    }
                }
            }
            else
            { // nilai setPoint2 diubah jadi positif
                btsSetPoint = setPoint2 + 360;

                if (angle >= btsSetPoint || angle <= setPoint1)
                {
                    PaMove = 0.00;
                    posRotateNew = true;
                }
                else
                {
                    btsRotate = rotate + 180;
                    if ((angle >= rotate) && (angle <= btsRotate))
                    {                                         // misal di range -180 - 0, maka putar kiri
                        putar = abs(rotate - angle) * 0.0065; // 0.0033
                        PaMove = putar;
                    }
                    else
                    { // putar kanan
                        if (angle < rotate)
                        {
                            putar = (rotate - angle) * 0.0065;
                        } // 0.0033
                        else
                        {
                            putar = ((180 + rotate) + (180 - angle)) * 0.0065;
                        } // 0.0033
                        PaMove = -putar;
                    }
                }
            }
        }
        else
        { // arah imu kedepan
            if (angle >= setPoint2 && angle <= setPoint1)
            {
                PaMove = 0.00;
                posRotateNew = true;
            }
            else
            {
                if (rotate >= 0)
                {
                    btsRotate = rotate - 180;
                    if ((angle <= rotate) && (angle >= btsRotate))
                    {                                      // putar kanan
                        putar = (rotate - angle) * 0.0065; // 0.0033
                        PaMove = -putar;
                    }
                    else
                    { // putar kiri
                        if (angle > rotate)
                        {
                            putar = (angle - rotate) * 0.0065;
                        } // 0.0033
                        else
                        {
                            putar = ((180 - rotate) + (180 + angle)) * 0.0065;
                        } // 0.0033
                        PaMove = putar;
                    }
                }
                else
                {
                    btsRotate = rotate + 180;
                    if ((angle >= rotate) && (angle <= btsRotate))
                    {                                         // maka putar kiri
                        putar = abs(rotate - angle) * 0.0065; // 0.0033
                        PaMove = putar;
                    }
                    else
                    { // putar kanan
                        if (angle < rotate)
                        {
                            putar = (rotate - angle) * 0.0065;
                        } // 0.0033
                        else
                        {
                            putar = ((180 + rotate) + (180 - angle)) * 0.0065;
                        } // 0.0033
                        PaMove = -putar;
                    }
                }
            }
        }
        if (!posRotateNew)
        {
            Walk(0.0, 0.0, PaMove);
        }
    }

    //	A+ = putar kiri
    //	A- = putar kanan
    void jalanDirection(double Xwalk, double Ywalk, double rotate)
    {
        if (rotate > 180)
        {
            rotate = 180;
        }
        else if (rotate < -180)
        {
            rotate = -180;
        }
        //-175 sampai -185
        setPoint1 = 5 + rotate;  // 10 //15 //20 //60
        setPoint2 = -5 + rotate; // 10 //15 //20 //60

        if (setPoint1 > 180 || setPoint2 < -180)
        { // jika arah imu dibelakang
            if (setPoint1 > 180)
            { // nilai setpoint1 diubah jd negatif
                btsSetPoint = setPoint1 - 360;

                if (angle >= setPoint2 || angle <= btsSetPoint)
                { // misal 170 ke -170
                    PaMove = 0.00;
                }
                else
                {
                    btsRotate = rotate - 180;
                    if ((angle <= rotate) && (angle >= btsRotate))
                    {                                      // misal di range 0 - 180, maka putar kanan
                        putar = (rotate - angle) * 0.0065; // 0.0033
                        PaMove = -putar;
                    }
                    else
                    { // putar kiri
                        if (angle > rotate)
                        {
                            putar = (angle - rotate) * 0.004;
                        } // 0.0033
                        else
                        {
                            putar = ((180 - rotate) + (180 + angle)) * 0.004;
                        } // 0.0033
                        PaMove = putar;
                    }
                }
            }
            else
            { // nilai setPoint2 diubah jadi positif
                btsSetPoint = setPoint2 + 360;

                if (angle >= btsSetPoint || angle <= setPoint1)
                {
                    PaMove = 0.00;
                }
                else
                {
                    btsRotate = rotate + 180;
                    if ((angle >= rotate) && (angle <= btsRotate))
                    {                                        // misal di range -180 - 0, maka putar kiri
                        putar = abs(rotate - angle) * 0.004; // 0.0033
                        PaMove = putar;
                    }
                    else
                    { // putar kanan
                        if (angle < rotate)
                        {
                            putar = (rotate - angle) * 0.004;
                        } // 0.0033
                        else
                        {
                            putar = ((180 + rotate) + (180 - angle)) * 0.004;
                        } // 0.0033
                        PaMove = -putar;
                    }
                }
            }
        }
        else
        { // arah imu kedepan
            if (angle >= setPoint2 && angle <= setPoint1)
            {
                PaMove = 0.00;
            }
            else
            {
                if (rotate >= 0)
                {
                    btsRotate = rotate - 180;
                    if ((angle <= rotate) && (angle >= btsRotate))
                    {                                     // putar kanan
                        putar = (rotate - angle) * 0.004; // 0.0033
                        PaMove = -putar;
                    }
                    else
                    { // putar kiri
                        if (angle > rotate)
                        {
                            putar = (angle - rotate) * 0.004;
                        } // 0.0033
                        else
                        {
                            putar = ((180 - rotate) + (180 + angle)) * 0.004;
                        } // 0.0033
                        PaMove = putar;
                    }
                }
                else
                {
                    btsRotate = rotate + 180;
                    if ((angle >= rotate) && (angle <= btsRotate))
                    {                                        // maka putar kiri
                        putar = abs(rotate - angle) * 0.004; // 0.0033
                        PaMove = putar;
                    }
                    else
                    { // putar kanan
                        if (angle < rotate)
                        {
                            putar = (rotate - angle) * 0.004;
                        } // 0.0033
                        else
                        {
                            putar = ((180 + rotate) + (180 - angle)) * 0.004;
                        } // 0.0033
                        PaMove = -putar;
                    }
                }
            }
        }

        if (PaMove > 0.3)
        {
            PaMove = 0.3;
        }
        else if (PaMove < -0.3)
        {
            PaMove = -0.3;
        }

        Walk(Xwalk, Ywalk, PaMove);
    }

    // Ball Positioning Using P Controller =======================================================
    double errorPosX,
        errorPosY,
        PxMoveBallPos,
        PyMoveBallPos,
        PaMoveBallPos;
    bool ballPos = false;
    void ballPositioning(double setPointX, double setPointY, double speed)
    {

        errorPosX = headPan - setPointX;
        errorPosY = headTilt - setPointY;

        if ((errorPosX > -0.2 && errorPosX < 0.2) && (errorPosY > -0.15))
        { //&& errorPosY < 0.10)) { //sudah sesuai
            PyMoveBallPos = 0.00;
            PxMoveBallPos = 0.00;
            ballPos = true;
        }
        else
        { // belum sesuai
            ballPos = false;
            if ((headPan >= 1.0 && headTilt >= -1.2) || (headPan <= -1.0 && headTilt >= -1.2))
            { // bola disamping //pan tilt kircok (polar)
                PxMoveBallPos = -0.03;
                PyMoveBallPos = errorPosX * 0.08; // 0.12;
            }
            else
            {
                // Xmove
                if (headTilt > setPointY)
                { //> (setPointY + 0.1)) { //kelebihan
                    PxMoveBallPos = -0.03;
                }
                else if (headTilt >= (setPointY - 0.1) && headTilt <= setPointY)
                { //<= (setPointY + 0.1)) { //sudah dalam range
                    PxMoveBallPos = 0.00;
                }
                else if (headTilt >= (setPointY - 0.3) && headTilt < (setPointY - 0.1))
                { // bola sudah dekat
                    PxMoveBallPos = errorPosY * -speed;
                    if (PxMoveBallPos >= 0.02)
                    {
                        PxMoveBallPos = 0.02;
                    }
                    else if (PxMoveBallPos <= 0.00)
                    {
                        PxMoveBallPos = 0.00;
                    }
                }
                else
                {                                             // bola masih jauh
                    PxMoveBallPos = headTilt * (0.06 / -1.6); // 0.05
                }

                // Ymove
                if (headTilt >= (setPointY - 0.03))
                { //> (setPointY + 0.1)) { //kelebihan
                    PyMoveBallPos = 0.00;
                }
                else
                {
                    if (headPan >= (setPointX - 0.1) && headPan <= (setPointX + 0.1))
                    { // sudah dalam range
                        PyMoveBallPos = 0.00;
                    }
                    else
                    {                                     // belum dalam range
                        PyMoveBallPos = errorPosX * 0.08; // 0.08;//0.12;
                    }
                }
            }
        }
        Walk(PxMoveBallPos, PyMoveBallPos, 0.0);
    }

    // Ball Positioning Using P Controller =======================================================
    double errorPosXz,
        errorPosYz,
        PxMoveBallPosz,
        PyMoveBallPosz,
        PaMoveBallPosz;
    // bool	ballPosz = false;
    void ballPositioningZero(double setPointX, double setPointY, double speed, int cudut)
    {
        // trackBall();
        errorPosXz = headPan - setPointX;
        errorPosYz = headTilt - setPointY;

        if ((errorPosXz > -0.1 && errorPosXz < 0.1) && (errorPosYz > -0.05))
        { //&& errorPosY < 0.10)) { //sudah sesuai
            PyMoveBallPosz = 0.00;
            PxMoveBallPosz = 0.00;
            ballPos = true;
        }
        else
        { // belum sesuai
            ballPos = false;
            if ((headPan >= 1.0 && headTilt >= -1.2) || (headPan <= -1.0 && headTilt >= -1.2))
            { // bola disamping //pan tilt kircok (polar)
                PxMoveBallPosz = 0.00;
                PyMoveBallPosz = errorPosXz * 0.08; // 0.07 0.12;
            }
            else
            {
                // Xmove
                if (headTilt > setPointY)
                { //> (setPointY + 0.1)) { //kelebihan
                    PxMoveBallPosz = -0.01;
                }
                else if (headTilt >= (setPointY - 0.05) && headTilt <= setPointY)
                { // bola sudah dekat
                    PxMoveBallPosz = -0.00;
                }
                else if (headTilt >= (setPointY - 0.3) && headTilt < (setPointY - 0.1))
                { // bola masih jauh
                    PxMoveBallPosz = errorPosYz * -speed;
                    if (PxMoveBallPosz >= 0.015)
                    {
                        PxMoveBallPosz = 0.015;
                    }
                    else if (PxMoveBallPosz <= 0.00)
                    {
                        PxMoveBallPosz = 0.00;
                    }
                }
                else
                {
                    PxMoveBallPosz = headTilt * (0.08 / -1.6); // 0.05
                }

                // Ymove
                if (headPan >= (setPointX - 0.1) && headPan <= (setPointX + 0.1))
                { // sudah dalam range
                    PyMoveBallPosz = 0.00;
                }
                else
                {                                       // belum dalam range
                    PyMoveBallPosz = errorPosXz * 0.08; // 0.08;//0.12;
                    // PxMoveBallPos = 0.0;
                }
            }
        }

        if (PxMoveBallPosz > 0.05)
        {
            PxMoveBallPosz = 0.05;
        }
        else if (PxMoveBallPosz < -0.02)
        {
            PxMoveBallPosz = -0.02;
        }

        if (PyMoveBallPosz > 0.02)
        {
            PyMoveBallPosz = 0.02;
        }
        else if (PyMoveBallPosz < -0.02)
        {
            PyMoveBallPosz = -0.02;
        }
        jalanDirection(PxMoveBallPosz, PyMoveBallPosz, cudut);
        // Walk(PxMoveBallPosz, PyMoveBallPosz, 0.0);
    }

    // Dribble Ball ======================================================================
    int bawaBola;
    double setPointFootY, setPointFootY1, setPointFootY2;
    void dribble(int gawang, double speed)
    {
        trackBall();

        sudut();

        // setPoint1 =  20 + gawang;//20
        // setPoint2 = -20 + gawang;//20

        // if (angle >= setPoint2 && angle <= setPoint1) {
        //	robotDirection = true;
        // } else  { robotDirection = false; }

        // if (robotDirection) { //printf("arah imu sudah benar\n");
        // if (headTilt <= -0.7 || bawaBola >= 300) {
        //	bawaBola = 0;
        //	robotDirection = false;
        //	stateCondition = 5;
        // } bawaBola++;

        // Ball Positioning ======================================
        // if (posPan >= 0) { setPointFootY = 0.25;  }//kiri
        // else             { setPointFootY = -0.25; }//kanan
        // errorPosX = headPan - setPointFootY;

        setPointFootY1 = 0.2;       // 0.22
        setPointFootY2 = -0.2;      //-0.22
        errorPosY = headTilt + 0.5; // 0.04;//0.05;//0.08;

        // if(errorPosX >= -0.1 && errorPosX <= 0.1) {
        if (headPan <= setPointFootY1 && headPan >= setPointFootY2)
        { // -0.2 > x < 0.2
            // if (headTilt >= -0.8) {
            PxMoveBallPos = 0.3 * speed;
            //} else {
            //	PxMoveBallPos = errorPosY*speed*-1;
            //}
            Walk(PxMoveBallPos, 0.0, 0.0);
        }
        else
        { // x < -0.2 || x > 0.2
            if (headTilt >= pTiltTendang)
            {
                PxMoveBallPos = errorPosY * -speed;
                // PxMoveBallPos = -0.02;//-0.03;
                // Walk(-0.03, 0.0, 0.0);
            }
            else
            {
                PxMoveBallPos = 0.0;
            }

            if (headPan > setPointFootY1)
            { // printf("kiri bos\n");
                PyMoveBallPos = (headPan - 0.1) * 0.06;
            }
            else if (headPan < setPointFootY2)
            { // printf("kanan bos\n");
                PyMoveBallPos = (headPan + 0.1) * 0.06;
            }

            Walk(PxMoveBallPos, PyMoveBallPos, 0.0);

            // if (headTilt >= (cSekarang - 0.2)) {
            //	Walk(-0.03, 0.0, 0.0);
            // } else {
            //	if (headPan > setPointFootY1) { //printf("kiri bos\n");
            //		PyMoveBallPos = (headPan - 0.1) * 0.06;
            //		Walk(0.0, PyMoveBallPos, 0.0);
            //	} else if (headPan < setPointFootY2) { //printf("kanan bos\n");
            //		PyMoveBallPos = (headPan + 0.1) * 0.06;
            //		Walk(0.0, PyMoveBallPos, 0.0);
            //	}
            // }

            // if(errorPosY >= -0.1) { // XmoveBackWard
            //	Walk(-0.03, 0.0, 0.0);
            // } else { // Ymove
            //	PyMoveBallPos = errorPosX*0.06;
            //	Walk(0.0, PyMoveBallPos, 0.0);
            // }
        }
        //} else { //printf("cari arah imu\n");
        //	bawaBola = 0;
        //	if (posTilt > -0.8 && posPan > -0.5 && posPan < 0.5) {//bola dekat
        //		Imu(gawang, cSekarang);
        //	} else {//bola masih jauh
        //		followBall(0);
        //	}
        //}
    }

    // Checking Lost Goal ========================================================================
    int countGoalLost = 0,
        countGoalFound = 0,
        returnGoalVal;
    int goalLost(int threshold)
    {
        if (useVision)
        {
            if (Goal_X == -1 && Goal_Y == -1)
            {
                countGoalFound = 0;
                countGoalLost++;
                if (countGoalLost >= threshold)
                {
                    returnGoalVal = 1;
                }
            }
            else
            {
                if (headTilt < -1.0)
                {
                    countGoalLost = 0;
                    countGoalFound++;
                    if (countGoalFound > 1)
                    {
                        returnGoalVal = 0;
                    }
                }
            }
        }
        else
        {
            countGoalFound = 0;
            countGoalLost++;
            if (countGoalLost >= threshold)
            {
                returnGoalVal = 1;
            }
        }
        return returnGoalVal;
    }

    // Goal Tracking =============================================================================
    double intPanG = 0, dervPanG = 0, errorPanG = 0, preErrPanG = 0,
           PPanG = 0, IPanG = 0, DPanG = 0,
           intTiltG = 0, dervTiltG = 0, errorTiltG = 0, preErrTiltG = 0,
           PTiltG = 0, ITiltG = 0, DTiltG = 0,
           dtG = 0.04;
    double G_Pan_err_diff, G_Pan_err, G_Tilt_err_diff, G_Tilt_err, G_PanAngle, G_TiltAngle,
        pOffsetG, iOffsetG, dOffsetG,
        errorPanGRad, errorTiltGRad;
    int offsetSetPointGoal;
    void trackGoal()
    {
        if (useVision)
        {
            if (Goal_X != -1 && Goal_Y != -1)
            { // printf("Tracking");
                // mode 1 ######################################################################
                // PID pan ==========================================================
                /*errorPanG  = (double)Goal_X - (frame_X / 2);//160
                PPanG  = errorPanG  * 0.00010; //Tune in Kp Pan  0.00035 //kalau kepala msh goyang2, kurangin nilainya

                intPanG += errorPanG * dtG;
                IPanG = intPanG * 0.0;

                dervPanG = (errorPanG - preErrPanG) / dtG;
                DPanG = dervPanG * 0.00001;

                preErrPanG = errorPanG;

                //posPan += PPanG*-1; //dikali -1 kalau receler terbalik dalam pemasangan
                posPan += (PPanG + IPanG + DPanG) * -1;


                //PID tilt ==========================================================
                errorTiltG = (double)Goal_Y - (frame_Y / 2);//120
                PTiltG = errorTiltG * 0.00010; //Tune in Kp Tilt 0.00030

                intTiltG += errorTiltG * dtG;
                ITiltG = intTiltG * 0.0;

                dervTiltG = (errorTiltG - preErrTiltG) / dtG;
                DTiltG = dervTiltG * 0; //0.00001;

                preErrTiltG = errorTiltG;

                //posTilt += PTiltG;
                posTilt += (PTiltG + ITiltG + DTiltG);*/

                // mode 2 ######################################################################
                //  offsetSetPointGoal = (int)((posTilt * 30) + 54);
                //  if (offsetSetPointGoal > 36) offsetSetPointGoal = 36;
                //  else if (offsetSetPointGoal < 0) offsetSetPointGoal = 0;

                // errorPanG  = (double)Goal_X - ((frame_X / 2) + offsetSetPointGoal);//160
                errorPanG = (double)Goal_X - (frame_X / 2);
                errorTiltG = (double)Goal_Y - (frame_Y / 2); // 120

                errorPanG *= -1;
                errorTiltG *= -1;
                errorPanG *= (90 / (double)frame_X);     // pixel per angle
                errorTiltG *= (60 / (double)frame_Y);    // pixel per angle
                errorPanG *= (77.32 / (double)frame_X);  // pixel per angle
                errorTiltG *= (61.93 / (double)frame_Y); // pixel per angle

                errorPanGRad = (errorPanG * PI) / 180;
                errorTiltGRad = (errorTiltG * PI) / 180;
                // printf("errorPan = %.2f \t errorTilt = %.2f\n", errorPanG, errorTiltG);
                // printf("RadrrorPan = %.2f \t RaderrorTilt = %.2f\n", errorPanGRad, errorTiltGRad);
                // printf("KPPan = %f \t KDPan = %f\t", kamera.panKP, kamera.panKD); printf("KPTilt = %f \t KDTilt = %f\n", kamera.tiltKP, kamera.tiltKD);

                G_Pan_err_diff = errorPanGRad - G_Pan_err;
                G_Tilt_err_diff = errorTiltGRad - G_Tilt_err;

                // PID pan ==========================================================
                // PPanG  = G_Pan_err  * kamera.panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
                PPanG = G_Pan_err * goal_panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
                intPanG += G_Pan_err * dtG;
                IPanG = intPanG * 0.0;
                dervPanG = G_Pan_err_diff / dtG;
                // DPanG = dervPanG * kamera.panKD;
                DPanG = dervPanG * goal_panKD;
                G_Pan_err = errorPanGRad;
                posPan += (PPanG + IPanG + DPanG);

                // PID tilt ==========================================================
                // PTiltG = G_Tilt_err * kamera.tiltKP; // Tune in Kp Tilt 0.00030
                PTiltG = G_Tilt_err * goal_tiltKP; // Tune in Kp Tilt 0.00030

                intTiltG += G_Tilt_err * dtG;
                ITiltG = intTiltG * 0.0;

                dervTiltG = G_Tilt_err_diff / dtG;
                // DTiltG = dervTiltG * kamera.tiltKD;
                DTiltG = dervTiltG * goal_tiltKD;

                preErrTiltG = errorTiltG;
                G_Tilt_err = errorTiltGRad;
                posTilt += (PTiltG + ITiltG + DTiltG) * -1;

                if (posPan >= 1.6)
                {
                    posPan = 1.6;
                }
                else if (posPan <= -1.6)
                {
                    posPan = -1.6;
                }
                if (posTilt <= -2.0)
                {
                    posTilt = -2.0;
                }
                else if (posTilt >= -0.4)
                {
                    posTilt = -0.4;
                }

                headMove(posPan, posTilt); // printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
            }
        }
    }

    // Follow Goal ===============================================================================
    int countReadyStop = 0;
    void followGoal(double Xwalk, double SetPan, int mode)
    { // 0 normal, 1 sambil belok
        errorfPan = posPan - SetPan;

        if (posTilt < -2.0 && posPan < 0.4 && posPan > -0.4 && Xcross_LX != -1 && Xcross_LY != -1)
        { // Stop
            countReadyStop++;
        }
        else
        { // Follow
            countReadyStop = 0;
        }

        if (countReadyStop >= 5)
        {
            PxMove = 0.00;              // jalan ditempat
            PyMove = errorfPan * 0.040; // 0.045
            PaMove = errorfPan * 0.20;  // 0.30; //0.045
        }
        else
        {
            PxMove = Xwalk;             // 0.08
            PyMove = errorfPan * 0.040; // 0.045
            PaMove = errorfPan * 0.20;  // 0.35; //0.045
        }

        if (mode == 0)
        { // pake alfa
            if (errorfPan > -0.4 && errorfPan < 0.4)
            { // printf("AAAAAAAA\n");
                Walk(PxMove, 0.0, PaMove);
            }
            else
            { // printf("BBBBBBBB\n");
                Walk(0.0, 0.0, PaMove);
            }
        }
        else if (mode == 1)
        { // tanpa alfa
            if (errorfPan > -0.4 && errorfPan < 0.4)
            { // printf("CCCCCCCC\n");
                Walk(PxMove, PyMove, 0.0);
            }
            else
            { // printf("DDDDDDDD\n");
                Walk(0.0, PyMove, 0.0);
            }
        }
    }

    // Body Tracking Goal ========================================================================
    double errorBodyPositionG,
        bodyP_ControllerG;
    int bodyTrueG = 0,
        delayTrueG = 0;
    int bodyTrackingGoal(int threshold)
    {
        //	trackGoal();

        errorBodyPositionG = 0 - headPan;
        bodyP_ControllerG = errorBodyPositionG * -0.5; //-0.5

        if (errorBodyPositionG >= -0.1 && errorBodyPositionG <= 0.1)
        { // untuk hasil hadap 0.8
            // motion("0");
            Walk(0.0, 0.0, 0.0);
            delayTrueG++;
        }
        else
        {
            trackGoal();
            // motion("9");

            bodyTrueG =
                delayTrueG = 0;

            if (bodyP_ControllerG < 0)
            { // kanan
                // Walk(rotateGoal_x, abs(rotateGoal_y), -abs(rotateGoal_a));
                Walk(rotateGoal_x, rotateGoal_y, -rotateGoal_a);
            }
            else
            { // kiri
                // Walk(rotateGoal_x, -abs(rotateGoal_y), abs(rotateGoal_a));
                Walk(rotateGoal_x, -rotateGoal_y, rotateGoal_a);
            }
        }

        if (delayTrueG >= threshold)
        {
            bodyTrueG = 1;
        }
        else
        {
            bodyTrueG = 0;
        }
        return bodyTrueG;
    }

    // Search Goal =====================================================================================================
    double goalPan = 0;
    void saveGoalLocation()
    {
        //	trackGoal();
        goalPan = headPan;
    }

    // Landmark Tracking =============================================================================
    double intPanL = 0, dervPanL = 0, errorPanL = 0, preErrPanL = 0,
           PPanL = 0, IPanL = 0, DPanL = 0,
           intTiltL = 0, dervTiltL = 0, errorTiltL = 0, preErrTiltL = 0,
           PTiltL = 0, ITiltL = 0, DTiltL = 0,
           dtL = 0.04;
    double L_Pan_err_diff, L_Pan_err, L_Tilt_err_diff, L_Tilt_err,
        errorPanLRad, errorTiltLRad;
    int offsetSetPointLand;
    void trackLand()
    {
        if (useVision)
        {
            if (Xcross_LX != -1 && Xcross_LY != -1)
            { // printf("Tracking");
                // mode 1 ######################################################################
                // PID pan ==========================================================
                /*errorPanL  = (double)Goal_X - (frame_X / 2);//160
                PPanL  = errorPanL  * 0.00010; //Tune in Kp Pan  0.00035 //kalau kepala msh goyang2, kurangin nilainya

                intPanL += errorPanL * dtL;
                IPanL = intPanL * 0.0;

                dervPanL = (errorPanL - preErrPanL) / dtL;
                DPanL = dervPanL * 0.00001;

                preErrPanL = errorPanL;

                //posPan += PPanL*-1; //dikali -1 kalau receler terbalik dalam pemasangan
                posPan += (PPanL + IPanL + DPanL) * -1;


                //PID tilt ==========================================================
                errorTiltL = (double)Goal_Y - (frame_Y / 2);//120
                PTiltL = errorTiltL * 0.00010; //Tune in Kp Tilt 0.00030

                intTiltL += errorTiltL * dtL;
                ITiltL = intTiltL * 0.0;

                dervTiltL = (errorTiltL - preErrTiltL) / dtL;
                DTiltL = dervTiltL * 0; //0.00001;

                preErrTiltL = errorTiltL;

                //posTilt += PTiltL;
                posTilt += (PTiltL + ITiltL + DTiltL);*/

                // mode 2 ######################################################################
                offsetSetPointLand = (int)((posTilt * 30) + 54);
                if (offsetSetPointLand > 36)
                    offsetSetPointLand = 36;
                else if (offsetSetPointLand < 0)
                    offsetSetPointLand = 0;

                if ((Xcross_LX != -1 && Xcross_LY != -1) && (Xcross_RX != -1 && Xcross_RY != -1))
                {
                    errorPanL = (double)Xcross_RX - ((frame_X / 2) + offsetSetPointLand); // 160
                    errorTiltL = (double)Xcross_RY - (frame_Y / 2);                       // 120
                }
                else
                {
                    errorPanL = (double)Xcross_LX - ((frame_X / 2) + offsetSetPointLand); // 160
                    errorTiltL = (double)Xcross_LY - (frame_Y / 2);                       // 120
                }

                errorPanL *= -1;
                errorTiltL *= -1;
                errorPanL *= (90 / (double)frame_X);     // pixel per angle
                errorTiltL *= (60 / (double)frame_Y);    // pixel per angle
                errorPanL *= (77.32 / (double)frame_X);  // pixel per angle
                errorTiltL *= (61.93 / (double)frame_Y); // pixel per angle

                errorPanLRad = (errorPanL * PI) / 180;
                errorTiltLRad = (errorTiltL * PI) / 180;
                // printf("errorPan = %.2f \t errorTilt = %.2f\n", errorPanL, errorTiltL);
                // printf("RadrrorPan = %.2f \t RaderrorTilt = %.2f\n", errorPanLRad, errorTiltLRad);
                // printf("KPPan = %f \t KDPan = %f\t", kamera.panKP, kamera.panKD); printf("KPTilt = %f \t KDTilt = %f\n", kamera.tiltKP, kamera.tiltKD);

                L_Pan_err_diff = errorPanLRad - L_Pan_err;
                L_Tilt_err_diff = errorTiltLRad - L_Tilt_err;

                // PID pan ==========================================================
                // PPanL  = L_Pan_err  * kamera.panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
                PPanL = L_Pan_err * goal_panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
                intPanL += L_Pan_err * dtL;
                IPanL = intPanL * 0.0;
                dervPanL = L_Pan_err_diff / dtL;
                // DPanL = dervPanL * kamera.panKD;
                DPanL = dervPanL * goal_panKD;
                L_Pan_err = errorPanLRad;
                posPan += (PPanL + IPanL + DPanL);

                // PID tilt ==========================================================
                // PTiltL = L_Tilt_err * kamera.tiltKP; // Tune in Kp Tilt 0.00030
                PTiltL = L_Tilt_err * goal_tiltKP; // Tune in Kp Tilt 0.00030

                intTiltL += L_Tilt_err * dtL;
                ITiltL = intTiltL * 0.0;

                dervTiltL = L_Tilt_err_diff / dtL;
                // DTiltL = dervTiltL * kamera.tiltKD;
                DTiltL = dervTiltL * goal_tiltKD;

                preErrTiltL = errorTiltL;
                L_Tilt_err = errorTiltLRad;
                posTilt += (PTiltL + ITiltL + DTiltL) * -1;

                if (posPan >= 1.6)
                {
                    posPan = 1.6;
                }
                else if (posPan <= -1.6)
                {
                    posPan = -1.6;
                }
                if (posTilt <= -2.0)
                {
                    posTilt = -2.0;
                }
                else if (posTilt >= -0.4)
                {
                    posTilt = -0.4;
                }

                headMove(posPan, posTilt); // printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
            }
        }
    }

    // Follow Landmark ===============================================================================
    int countReadyStopL = 0;
    void followLand(double Xwalk, double SetPan, int mode)
    { // 0 normal, 1 sambil belok
        errorfPan = posPan - SetPan;

        if (posTilt < -2.0 && posPan < 0.4 && posPan > -0.4 && Xcross_LX != -1 && Xcross_LY != -1)
        { // Stop
            countReadyStopL++;
        }
        else
        { // Follow
            countReadyStopL = 0;
        }

        if (countReadyStopL >= 5)
        {
            PxMove = 0.00;              // jalan ditempat
            PyMove = errorfPan * 0.040; // 0.045
            PaMove = errorfPan * 0.20;  // 0.30; //0.045
        }
        else
        {
            PxMove = Xwalk;             // 0.08
            PyMove = errorfPan * 0.040; // 0.045
            PaMove = errorfPan * 0.20;  // 0.35; //0.045
        }

        if (mode == 0)
        { // pake alfa
            if (errorfPan > -0.4 && errorfPan < 0.4)
            { // printf("AAAAAAAA\n");
                Walk(PxMove, 0.0, PaMove);
            }
            else
            { // printf("BBBBBBBB\n");
                Walk(0.0, 0.0, PaMove);
            }
        }
        else if (mode == 1)
        { // tanpa alfa
            if (errorfPan > -0.4 && errorfPan < 0.4)
            { // printf("CCCCCCCC\n");
                Walk(PxMove, PyMove, 0.0);
            }
            else
            { // printf("DDDDDDDD\n");
                Walk(0.0, PyMove, 0.0);
            }
        }
    }

    void saveSudutImu()
    {
        //	sudut();
        saveAngle = angle;
    }

    int prediksiGoalPan = 0;
    double Rotate = 0;
    void prediksiArahGoal()
    {
        prediksiGoalPan = (int)(57.29 * headPan);
        Rotate = headPan * (8 / 1.57); // 90 derajat = 8 detik //waktu rotate
    }

    int count = 0;
    bool goalSearch = false,
         searchGoalFinish = false;
    void panSearchGoal(double arah)
    { // printf("  panSearchBall\n\n");
        if (panRate < 0)
        {
            panRate = -panRate;
        }

        if (arah < 0)
        {
            headPan += panRate;
            if (headPan >= batasKiri)
            {
                prediksiGoalPan = 0;
                saveAngle = 0;
                Rotate = 0;
                goalSearch = true;
            }
        }
        else
        {
            headPan -= panRate;
            if (headPan <= batasKanan)
            {
                prediksiGoalPan = 0;
                saveAngle = 0;
                Rotate = 0;
                goalSearch = true;
            }
        }

        if (headPan >= batasKiri)
        {
            headPan = batasKiri;
        }
        else if (headPan <= batasKanan)
        {
            headPan = batasKanan;
        }

        headMove(headPan, -2.0); // printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
    }

    // +predictPanGoal = kiri
    // -predictPanGoal = kanan

    //	A+ = putar kiri
    //	A- = putar kanan
    double predictGoalPan;
    void predictGoal(double alpha, double tilt)
    {
        predictGoalPan = alpha / 57.29; // sudut * nilai per satu sudut(nilai servo)

        if (predictGoalPan <= -1.6)
        {
            predictGoalPan = -1.6;
        }
        else if (predictGoalPan >= 1.6)
        {
            predictGoalPan = 1.6;
        }
        // printf("  predict = %f\n\n",predictGoalPan);

        headMove(predictGoalPan, tilt);
    }

    void predictGoalTeam(double alpha, double tilt)
    {
        int setPointPan1, setPointPan2,
            btsSetPointPan, btsRotatePan;
        double putarPan;

        if (alpha > 180)
        {
            alpha = 180;
        }
        else if (alpha < -180)
        {
            alpha = -180;
        }

        setPointPan1 = 5 + alpha;
        setPointPan2 = -5 + alpha;

        if (setPointPan1 > 180 || setPointPan2 < -180)
        { // jika arah imu dibelakang
            if (setPointPan1 > 180)
            { // nilai setPointPan1 diubah jd negatif
                btsSetPointPan = setPointPan1 - 360;

                if (angle >= setPointPan2 || angle <= btsSetPointPan)
                { // misal 170 ke -170
                    predictGoalPan = 0.00;
                }
                else
                {
                    btsRotatePan = alpha - 180;
                    if ((angle <= alpha) && (angle >= btsRotatePan))
                    { // misal di range 0 - 180, maka putarPan kanan
                        putarPan = (alpha - angle) / 57.29;
                        predictGoalPan = -putarPan;
                    }
                    else
                    { // putarPan kiri
                        if (angle > alpha)
                        {
                            putarPan = (angle - alpha) / 57.29;
                        } // 0.0033
                        else
                        {
                            putarPan = ((180 - alpha) + (180 + angle)) / 57.29;
                        } // 0.0033
                        predictGoalPan = putarPan;
                    }
                }
            }
            else
            { // nilai setPointPan2 diubah jadi positif
                btsSetPointPan = setPointPan2 + 360;

                if (angle >= btsSetPointPan || angle <= setPointPan1)
                {
                    predictGoalPan = 0.00;
                }
                else
                {
                    btsRotatePan = alpha + 180;
                    if ((angle >= alpha) && (angle <= btsRotatePan))
                    {                                          // misal di range -180 - 0, maka putarPan kiri
                        putarPan = abs(alpha - angle) / 57.29; // 0.0033
                        predictGoalPan = putarPan;
                    }
                    else
                    { // putarPan kanan
                        if (angle < alpha)
                        {
                            putarPan = (alpha - angle) / 57.29;
                        } // 0.0033
                        else
                        {
                            putarPan = ((180 + alpha) + (180 - angle)) / 57.29;
                        } // 0.0033
                        predictGoalPan = -putarPan;
                    }
                }
            }
        }
        else
        { // arah imu kedepan
            if (angle >= setPointPan2 && angle <= setPointPan1)
            {
                predictGoalPan = 0.00;
            }
            else
            {
                if (alpha >= 0)
                {
                    btsRotatePan = alpha - 180;
                    if ((angle <= alpha) && (angle >= btsRotatePan))
                    {                                       // putarPan kanan
                        putarPan = (alpha - angle) / 57.29; // 0.0033
                        predictGoalPan = -putarPan;
                    }
                    else
                    { // putarPan kiri
                        if (angle > alpha)
                        {
                            putarPan = (angle - alpha) / 57.29;
                        } // 0.0033
                        else
                        {
                            putarPan = ((180 - alpha) + (180 + angle)) / 57.29;
                        } // 0.0033
                        predictGoalPan = putarPan;
                    }
                }
                else
                {
                    btsRotatePan = alpha + 180;
                    if ((angle >= alpha) && (angle <= btsRotatePan))
                    { // maka putarPan kiri
                        putarPan = abs(alpha - angle) / 57.29;
                        ; // 0.0033
                        predictGoalPan = putarPan;
                    }
                    else
                    { // putarPan kanan
                        if (angle < alpha)
                        {
                            putarPan = (alpha - angle) / 57.29;
                        } // 0.0033
                        else
                        {
                            putarPan = ((180 + alpha) + (180 - angle)) / 57.29;
                        } // 0.0033
                        predictGoalPan = -putarPan;
                    }
                }
            }
        }

        if (predictGoalPan <= -1.6)
        {
            predictGoalPan = -1.6;
        }
        else if (predictGoalPan >= 1.6)
        {
            predictGoalPan = 1.6;
        }

        headMove(predictGoalPan, tilt);
    }

    int goalSide = 0;
    double arahPandang = 0;
    int cekArah()
    {
        arahPandang = angle - (57.29 * headPan);
        if (arahPandang >= -90 && arahPandang <= 90)
        { // gawang lawan
            goalSide = 0;
        }
        else
        { // gawang sendiri
            goalSide = 1;
        }
        return goalSide;
    }

    int errorGoal = 0;
    double predictRotate;
    void searchGoal(int mode)
    {
        if (goalSearch)
        { // printf("\n\ncari bola\n\n"); //kedua
            if (tunggu < 10)
            {
                loadBallLocation(0.0);
                elapsedTime = 0;
                second = 0;
                reset = 0;
                robotDirection = false;
                timer = false;
                waiting = 0;
            }
            else if (tunggu > 50)
            {
                if (ballLost(20))
                {
                    if (waiting > 40)
                    {
                        resetCase1();
                        stateCondition = 1;
                    }
                    else
                    {
                        panSearchBall(cSekarang);
                        // tiltSearchBall(0.0);
                        Walk(0.0, 0.0, 0.0);
                    }
                    waiting++;
                }
                else
                {
                    trackBall();
                    waiting = 0;

                    if (mode == 1)
                    { // rotate dgn trackgoal
                        searchGoalFinish = true;
                    }
                    else if (mode == 2)
                    { // rotate dgn timer
                        if (reset > 5)
                        {
                            cekWaktu(abs(Rotate));
                            if (timer)
                            {
                                Walk(0.0, 0.0, 0.0);
                                searchGoalFinish = true;
                            }
                            else
                            {
                                if (Rotate < 0)
                                { // kiri
                                    rotateParabolic(1, cSekarang);
                                }
                                else
                                { // kanan
                                    rotateParabolic(-1, cSekarang);
                                }
                            }
                        }
                        else
                        {
                            setWaktu();
                            reset++;
                        }
                    }
                    else if (mode == 3)
                    { // rotate dgn offset Imu
                        if (robotDirection && headPan >= -0.4 && headPan <= 0.4)
                        {
                            Walk(0.0, 0.0, 0.0);
                            searchGoalFinish = true;
                        }
                        else
                        {
                            if (headTilt >= cAktif && headPan >= -0.4 && headPan <= 0.4)
                            {
                                if (reset > 5)
                                {
                                    cekWaktu(20);
                                    if (timer)
                                    {
                                        Walk(0.0, 0.0, 0.0);
                                        searchGoalFinish = true;
                                    }
                                    else
                                    {
                                        if (((saveAngle - prediksiGoalPan) > -30) && ((saveAngle - prediksiGoalPan) < 30))
                                        {
                                            if (prediksiGoalPan > 0)
                                            {
                                                errorGoal = -abs((saveAngle - prediksiGoalPan) * 0.4);
                                            } // 0.3 //0.33 //0.4
                                            else
                                            {
                                                errorGoal = abs((saveAngle - prediksiGoalPan) * 0.4);
                                            } // 0.3 //0.33 //0.4
                                            if (errorGoal >= 20)
                                            {
                                                errorGoal = 20;
                                            }
                                        }
                                        else if (((saveAngle - prediksiGoalPan) >= 30) && ((saveAngle - prediksiGoalPan) < 60) || ((saveAngle - prediksiGoalPan) > -60) && ((saveAngle - prediksiGoalPan) <= -30))
                                        {
                                            if (prediksiGoalPan > 0)
                                            {
                                                errorGoal = -abs((saveAngle - prediksiGoalPan) * 0.3);
                                            } // 0.3 //0.33 //0.4
                                            else
                                            {
                                                errorGoal = abs((saveAngle - prediksiGoalPan) * 0.3);
                                            } // 0.3 //0.33 //0.4
                                            if (errorGoal >= 20)
                                            {
                                                errorGoal = 20;
                                            }
                                        }
                                        else
                                        {
                                            errorGoal = 0;
                                        }

                                        if (useSideKick)
                                        {
                                            if (prediksiGoalPan >= 45)
                                            {                 // kiri
                                                modeKick = 4; // tendangSamping
                                                Imu(90 + (saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
                                            }
                                            else if (prediksiGoalPan <= -45)
                                            {                 // kanan
                                                modeKick = 3; // tendangSamping
                                                Imu(-90 + (saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
                                            }
                                            else
                                            {
                                                modeKick = tendangJauh;
                                                Imu((saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
                                                lastDirection = angle;
                                            }
                                        }
                                        else
                                        {
                                            modeKick = tendangJauh;
                                            Imu((saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
                                            lastDirection = angle;
                                        }
                                    }
                                }
                                else
                                {
                                    setWaktu();
                                    robotDirection = false;
                                    reset++;
                                }
                            }
                            else
                            {
                                reset = 0;
                                followBall(0);
                            }
                        }
                    }
                }
            }
            else
            {
                elapsedTime = 0;
                second = 0;
                reset = 0;
                robotDirection = false;
                timer = false;
                waiting = 0;

                if (ballLost(20))
                {
                    panSearchBall(cSekarang);
                    // tiltSearchBall(0.0);
                }
                else
                {
                    trackBall();
                }
                Walk(0.0, 0.0, 0.0);
            }
            tunggu++;
        }
        else
        { // pertama
            tunggu = 0;

            if (goalLost(20))
            { // printf("\n\n  cari gawang\n\n");//fungsi yg cari gawang disini
                Walk(0.0, 0.0, 0.0);

                if (delay > 5)
                {
                    cekWaktu(2.5);
                    if (!timer)
                    {
                        if (second <= 2)
                        {
                            if (strategyCM == 3 || strategyCM == 4 || strategyCM == 9 || strategyCM == 10 || strategyCM == 11 || strategyCM == 12)
                            {
                                headMove(0.0, -2.0);
                            }
                            else
                            {
                                predictGoal(angle, -2.0);
                            }
                        }
                        else if (second > 2)
                        { //} && second <= 2.5) {
                            if (angle < 0)
                            {
                                headMove(-1.6, -2.0);
                            }
                            else
                            {
                                headMove(1.6, -2.0);
                            }
                        }
                    }
                    else
                    {
                        panSearchGoal(angle);
                        // printf("pan*** \n\n");
                    }
                }
                else
                {
                    setWaktu();
                    count = 0;
                    delay++;
                }
            }
            else
            { // printf("\n\n  rotate gawang\n\n");
                trackGoal();

                if (count > 50)
                { // 50 //60
                    if (((saveAngle - prediksiGoalPan) >= -90) && ((saveAngle - prediksiGoalPan) <= 90))
                    {
                        if (mode == 1)
                        {
                            if (bodyTrueG == 1)
                            {
                                Walk(0.0, 0.0, 0.0);
                                goalSearch = true;
                            }
                            else
                            {
                                bodyTrackingGoal(10); // 20
                            }
                        }
                        else if (mode == 2)
                        {
                            goalSearch = true;
                        }
                        else if (mode == 3)
                        {
                            goalSearch = true;
                        }
                    }
                    else
                    { // gawang sendiri
                        Walk(0.0, 0.0, 0.0);

                        prediksiGoalPan = 0;
                        saveAngle = 0;
                        Rotate = 0;

                        goalSearch = true;
                    }
                }
                else
                {
                    Walk(0.0, 0.0, 0.0);

                    prediksiArahGoal();
                    saveSudutImu();
                    // saveGoalLocation();

                    count++;
                }
            }
        }
    }

    bool rotateGoal = false,
         ballposTracked = false;
    void rotateToGoal(int mode)
    {
        if (!searchGoalFinish)
        { // pertama
            if (ballposTracked)
            {                  // printf("  searchgoal,"); //second
                searchGoal(3); // 1 2 3
            }
            else
            { // printf("  positioning,"); //first
                if (ballLost(20))
                { // 20
                    resetCase1();
                    stateCondition = 1;
                }
                else
                {
                    trackBall();
                    if (mode == 1)
                    {
                        if (headTilt >= (cSekarang - 0.2) && headPan >= -0.4 && headPan <= 0.4)
                        {
                            //						saveBallLocation();
                            ballposTracked = true;
                        }
                        else
                        {
                            followBall(0);
                        }
                    }
                    else if (mode == 2)
                    { //
                        if (ballPos)
                        { // positioning sebelum search goal
                            //						saveBallLocation();
                            ballposTracked = true;
                        }
                        else
                        {
                            ballPositioning(0.0, cSekarang, ballPositioningSpeed); // 0.15
                        }
                    }
                }
            }
        }
        else
        { // printf("  finish.........................................,"); //kedua
            trackBall();
            rotateGoal = true;
        }
    }

    // Tendang ===================================================================================
    bool tendang = false;
    void kick(int mode)
    {
        //	trackBall();
        if (mode == 3 || mode == 4)
        {
            if (mode == 3)
            {
                kanan = true;
                kiri = false;
            } // arah kanan
            else if (mode == 4)
            {
                kiri = true;
                kanan = false;
            } // arah kiri
        }
        else
        {
            if (posPan >= 0 && kanan == false && kiri == false)
            { // kiri
                kiri = true;
                kanan = false;
            }
            else if (posPan <= 0 && kanan == false && kiri == false)
            { // kanan
                kanan = true;
                kiri = false;
            }
        }

        if (kiri)
        { // kiri
            // if (posPan >= 0) { //kiri
            if (ballPos)
            { // printf("ball pos left true\n");
                motion("0");
                if (mode == 1 || mode == 2)
                {
                    sleep(1);
                    motion("1");
                }
                else if (mode == 3 || mode == 4)
                {
                    sleep(1); // 10
                    motion("4");
                }
                else if (mode == 5 || mode == 6)
                {
                    sleep(1);
                    motion("5");
                }
                tendang = true;
            }
            else
            {
                ballPositioning(-(pPanTendang), pTiltTendang, ballPositioningSpeed); // 0.15
            }
        }
        if (kanan)
        { // kanan
            //} else { //kanan
            if (ballPos)
            { // printf("ball pos right true\n");
                motion("0");
                if (mode == 1 || mode == 2)
                {
                    sleep(1); // 7
                    motion("2");
                }
                else if (mode == 3 || mode == 4)
                {
                    sleep(1); // 10
                    motion("3");
                }
                else if (mode == 5 || mode == 6)
                {
                    sleep(1); // 7
                    motion("6");
                }
                tendang = true;
            }
            else
            {
                ballPositioning(pPanTendang, pTiltTendang, ballPositioningSpeed); // 0.15
            }
        }
    }

    void rotateKickOffImu(int sudut, int mode)
    {
        if (headTilt >= -1.4)
        {
            if (robotDirection && headPan >= -0.4 && headPan <= 0.4)
            {
                if (sudut >= 0)
                { // kiri
                    if (ballPos)
                    { // printf("ball pos left true\n");
                        motion("0");
                        if (mode == 1 || mode == 2)
                        {
                            sleep(1); // 6
                            motion("1");
                        }
                        else if (mode == 3 || mode == 4)
                        {
                            sleep(1); // 10
                            motion("3");
                        }
                        else if (mode == 5 || mode == 6)
                        {
                            sleep(1);
                            motion("5");
                        }
                        tendang = true;
                    }
                    else
                    {
                        ballPositioning(-pPanOper, pTiltOper, ballPositioningSpeed); // 0.15
                    }
                }
                else
                { // kanan
                    if (ballPos)
                    { // printf("ball pos right true\n");
                        motion("0");
                        if (mode == 1 || mode == 2)
                        {
                            sleep(1); // 7
                            motion("2");
                        }
                        else if (mode == 3 || mode == 4)
                        {
                            sleep(1); // 10
                            motion("4");
                        }
                        else if (mode == 5 || mode == 6)
                        {
                            sleep(1); // 7
                            motion("6");
                        }
                        tendang = true;
                    }
                    else
                    {
                        ballPositioning(pPanOper, pTiltOper, ballPositioningSpeed); // 0.15
                    }
                }
            }
            else
            {
                if (headTilt >= (cAktif + 0.2) && headPan >= -0.4 && headPan <= 0.4)
                {                                 // +
                    Imu(sudut, cSekarang - 0.10); //- 0.20
                }
                else
                {
                    robotDirection = false;
                    followBall(0);
                }
            }
        }
        else
        {
            if (posTilt >= SetPointTilt)
            {
                posTilt = SetPointTilt;
            }
            else if (posTilt < -2.0)
            {
                posTilt = -2.0;
            }

            errorfPan = posPan - SetPointPan;
            errorfTilt = posTilt - SetPointTilt;

            if (posTilt >= SetPointTilt && posPan < 0.4 && posPan > -0.4 && Ball_X != -1 && Ball_Y != -1)
            {                               // Stop(bola sudah dekat)
                PxMove = 0.0;               // jalan ditempat
                PyMove = errorfPan * 0.040; // 0.045
                PaMove = errorfPan * 0.20;  // 0.30; //0.045
            }
            else
            {                               // Kejar Bola(bola masih jauh)
                PxMove = kejarMax;          // 0.06
                PyMove = errorfPan * 0.045; // 0.045
                PaMove = errorfPan * 0.25;  // 0.35; //0.045
            }

            if (errorfPan > -0.4 && errorfPan < 0.4)
            { // printf("AAAAAAAA\n");
                Walk(PxMove, 0.0, PaMove);
            }
            else
            { // printf("BBBBBBBB\n");
                Walk(0.0, 0.0, PaMove);
            }
            // followBall(0);
        }
    }

    void rotateKickOff(double timeRotate, int mode)
    {
        trackBall();

        // if (headTilt >= -1.0) {
        if (headTilt >= -0.8 && headPan >= -0.4 && headPan <= 0.4)
        {
            if (reset > 5)
            { // printf("set......................!!!\n");
                cekWaktu(abs(timeRotate));
                if (timer)
                { // printf("true......................!!!\n");
                    if (posPan >= 0)
                    { // kiri
                        if (ballPos)
                        { // printf("ball pos left true\n");
                            motion("0");
                            if (mode == 1 || mode == 2)
                            {
                                sleep(2); // 6
                                motion("1");
                            }
                            else if (mode == 3 || mode == 4)
                            {
                                // usleep(1000000); //10
                                sleep(2); // 10
                                motion("3");
                                // motion("4");
                            }
                            else if (mode == 5 || mode == 6)
                            {
                                sleep(2);
                                motion("5");
                            }
                            tendang = true;
                        }
                        else
                        {
                            ballPositioning(pPanOper, pTiltOper, ballPositioningSpeed); // 0.15
                        }
                    }
                    else
                    { // kanan
                        if (ballPos)
                        { // printf("ball pos right true\n");
                            motion("0");
                            if (mode == 1 || mode == 2)
                            {
                                sleep(2); // 7
                                motion("2");
                            }
                            else if (mode == 3 || mode == 4)
                            {
                                // usleep(1000000); //10
                                sleep(2); // 10
                                // motion("4");
                                motion("4");
                            }
                            else if (mode == 5 || mode == 6)
                            {
                                sleep(2); // 7
                                motion("6");
                            }
                            tendang = true;
                        }
                        else
                        {
                            ballPositioning(-pPanOper, pTiltOper, ballPositioningSpeed); // 0.15
                        }
                    }
                }
                else
                {
                    motion("9");
                    // rotateParabolic(timeRotate, cSekarang);
                    if (timeRotate < 0)
                    { // kanan
                        Walk(rotateGoal_x, rotateGoal_y, -rotateGoal_a);
                    }
                    else
                    { // kiri
                        Walk(rotateGoal_x, -rotateGoal_y, rotateGoal_a);
                    }
                }
            }
            else
            { // printf("cek......................!!!\n");
                motion("9");
                Walk(0.0, 0.0, 0.0);
                setWaktu();
                reset++;
            }
        }
        else
        {
            reset = 0;
            followBall(0);
        }
    }

    void kickNoSudut(int mode)
    {
        //   trackBall();
        // headMove(0.0,-1.3);
        //  if(headTilt >= -1.4){
        //  	if (robotDirection) {
        //  		if (ballPos) { //printf("ball pos right true\n");
        //  			motion("0");
        //  			headMove(0.0,-1.3);
        //  			if (mode == 4) {
        //  				//usleep(300000);
        //  				sleep(2); //7
        //  				motion("4");
        //  			} else if (mode == 1) {
        //  				// usleep(300000); //10
        //  				//usleep(1000000);
        //  				sleep(1); //10
        //  				//motion("4");
        //  				motion("1");
        //  			} else if (mode == 2) {
        //  				//usleep(300000); //10
        //  				sleep(1); //10
        //  				//motion("4");
        //  				motion("2");
        //  			}else if (mode == 3) {
        //  				//usleep(300000); //10
        //  				sleep(2); //10
        //  				//motion("4");
        //  				motion("3");
        //  			} else if (mode == 5) {
        //  				// usleep(300000); //10
        //  				sleep(1); //7
        //  				motion("5");
        //  			} else if (mode == 6) {
        //  				sleep(1); //7
        //  				// usleep(1); //10
        //  				motion("6");
        //  			} else if (mode == 4) {
        //  				sleep(2); //7
        //  				// usleep(300000); //10
        //  				motion("4");
        //  			}
        //  			tendang = true;
        //  		} else {
        //  			if (mode == 6 || mode == 2 || mode == 3){
        //  				ballPositioning(-pPanTendang, pTiltTendang, ballPositioningSpeed); //0.15
        //  				// ballPositioningZero(-pPanTendang, pTiltTendang, ballPositioningSpeed,modeImu); //0.15
        //  			} else if (mode == 1 || mode == 5 || mode == 4) {
        //  				// ballPositioningZero(pPanTendang + 0.02, pTiltTendang - 0.1, ballPositioningSpeed,modeImu); //0.15
        //  				// ballPositioningZero(pPanTendang, pTiltTendang, ballPositioningSpeed,modeImu); //0.15
        //  				ballPositioning(-pPanTendang, pTiltTendang, ballPositioningSpeed); //0.15
        //  			}
        //  		}
        //  	} else {
        //  		if (headPan >= -0.3 && headPan <= 0.3) {
        //  			//Imu(sudut, cSekarang - 0.20);
        //  			robotDirection = true;
        //  		} else {
        //  			robotDirection = false;
        //  			followBall(0);
        //  		}
        //  	}
        //  } else {
        //  	followBall(0);
        //  }
        if (headTilt >= -1.5)
        {
            if (robotDirection && headPan >= -0.4 && headPan <= 0.4)
            {
                if (ballPos)
                { // printf("ball pos left true\n");
                    motion("0");
                    if (mode == 1 || mode == 2)
                    {
                        usleep(900000); // 6
                        motion("2");
                    }
                    else if (mode == 3 || mode == 4)
                    {
                        usleep(300000); // 10
                        sleep(1);       // 10
                        motion("4");
                        // motion("4");
                    }
                    else if (mode == 5 || mode == 6)
                    {
                        usleep(850000);
                        motion("6");
                    }
                    tendang = true;
                }
                else
                {
                    ballPositioning(-pPanOper, pTiltOper, ballPositioningSpeed); // 0.15
                }
            }
            else
            {
                if (headTilt >= (cAktif + 0.2) && headPan >= -0.4 && headPan <= 0.4)
                {
                    // Imu(sudut, cSekarang - 0.20);
                    robotDirection = true;
                }
                else
                {
                    robotDirection = false;
                    followBall(0);
                }
            }
        }
        else
        {
            if (posTilt >= SetPointTilt)
            {
                posTilt = SetPointTilt;
            }
            else if (posTilt < -2.0)
            {
                posTilt = -2.0;
            }

            errorfPan = posPan - SetPointPan;
            errorfTilt = posTilt - SetPointTilt;

            if (posTilt >= SetPointTilt && posPan < 0.4 && posPan > -0.4 && Ball_X != -1 && Ball_Y != -1)
            {                               // Stop(bola sudah dekat)
                PxMove = 0.0;               // jalan ditempat
                PyMove = errorfPan * 0.040; // 0.045
                PaMove = errorfPan * 0.20;  // 0.30; //0.045
            }
            else
            {                               // Kejar Bola(bola masih jauh)
                PxMove = kejarMax;          // 0.06
                PyMove = errorfPan * 0.045; // 0.045
                PaMove = errorfPan * 0.25;  // 0.35; //0.045
            }

            if (errorfPan > -0.4 && errorfPan < 0.4)
            { // printf("AAAAAAAA\n");
                Walk(PxMove, 0.0, PaMove);
            }
            else
            { // printf("BBBBBBBB\n");
                Walk(0.0, 0.0, PaMove);
            }
            // followBall(0);
        }
    }

    int cntCheckGoal, cntCheckBall, cntRotate = 0;
    bool kickNow, rotateNow = false;
    void kickCheckGoal(int mode)
    {
        //	trackBall();
        if (mode == 3 || mode == 4)
        {
            if (mode == 3)
            {
                kanan = true;
                kiri = false;
            } // arah kanan
            else if (mode == 4)
            {
                kiri = true;
                kanan = false;
            } // arah kiri
        }
        else
        {
            if (posPan >= 0 && kanan == false && kiri == false)
            { // kiri
                kiri = true;
                kanan = false;
            }
            else if (posPan <= 0 && kanan == false && kiri == false)
            { // kanan
                kanan = true;
                kiri = false;
            }
        }

        if (kiri)
        { // kiri
            // if (posPan >= 0) { //kiri
            if (ballPos)
            { // printf("ball pos left true\n");
                if (cntCheckGoal > 20)
                {
                    if (kickNow)
                    {
                        motion("0");
                        if (mode == 1 || mode == 2)
                        {
                            sleep(1);
                            motion("1");
                        }
                        else if (mode == 3 || mode == 4)
                        {
                            sleep(1); // 10
                            motion("4");
                        }
                        else if (mode == 5 || mode == 6)
                        {
                            sleep(1);
                            motion("5");
                        }
                        tendang = true;
                    }
                    else
                    {
                        if (rotateNow)
                        {
                            if (angle > saveAngle - 5 && angle < saveAngle + 5)
                            {
                                kickNow = true;
                            }
                            else
                            {
                                if (ballLost(20))
                                {
                                    tracked = false;
                                }
                                else
                                {
                                    tracked = true;
                                }

                                if (tracked)
                                {
                                    trackBall();
                                    motion("9");
                                    if (cntRotate > 10)
                                    {
                                        Walk(0.0, -0.025, 0.2);
                                    }
                                    else
                                    {
                                        Walk(0.0, 0.0, 0.0);
                                        cntRotate++;
                                    }
                                }
                                else
                                {
                                    // headMove(0.16, -0.54);
                                    robotDirection = false;
                                }
                            }
                        }
                        else
                        {
                            if (GoalLeft_X < 400 || GoalLeft_X == -1)
                            {
                                kickNow = true;
                                rotateNow = false;
                            }
                            else if (GoalLeft_X > 400)
                            {
                                rotateNow = true;
                                kickNow = false;
                            }
                        }
                    }
                }
                else
                {
                    Walk(0.0, 0.0, 0.0);
                    motion("0");
                    headMove(0.0, -1.6);
                    // saveSudutImu();
                    cntCheckGoal++;
                }
            }
            else
            {
                trackBall();
                ballPositioning(pPanTendang, pTiltTendang, ballPositioningSpeed); // 0.15
            }
        }
        if (kanan)
        { // kanan
            //} else { //kanan
            if (ballPos)
            { // printf("ball pos left true\n");
                if (cntCheckGoal > 20)
                {
                    if (kickNow)
                    {
                        motion("0");
                        if (mode == 1 || mode == 2)
                        {
                            sleep(1);
                            motion("2");
                        }
                        else if (mode == 3 || mode == 4)
                        {
                            sleep(1); // 10
                            motion("3");
                        }
                        else if (mode == 5 || mode == 6)
                        {
                            sleep(1);
                            motion("6");
                        }
                        tendang = true;
                    }
                    else
                    {
                        if (rotateNow)
                        {
                            if (angle > saveAngle - 5 && angle < saveAngle + 5)
                            {
                                kickNow = true;
                            }
                            else
                            {
                                if (ballLost(20))
                                {
                                    tracked = false;
                                }
                                else
                                {
                                    tracked = true;
                                }

                                if (tracked)
                                {
                                    trackBall();
                                    motion("9");
                                    if (cntRotate > 10)
                                    {
                                        Walk(0.0, 0.025, -0.2);
                                    }
                                    else
                                    {
                                        Walk(0.0, 0.0, 0.0);
                                        cntRotate++;
                                    }
                                }
                                else
                                {
                                    tiltSearchBall(0.0);
                                    // headMove(-0.16, -0.54);
                                    robotDirection = false;
                                }
                            }
                        }
                        else
                        {
                            if (GoalRight_X > 400 || GoalRight_X == -1)
                            {
                                kickNow = true;
                                rotateNow = false;
                            }
                            else if (GoalRight_X < 400)
                            {
                                rotateNow = true;
                                kickNow = false;
                            }
                        }
                    }
                }
                else
                {
                    Walk(0.0, 0.0, 0.0);
                    motion("0");
                    headMove(0.0, -1.6);
                    saveSudutImu();
                    cntCheckGoal++;
                }
            }
            else
            {
                trackBall();
                ballPositioning(-pPanTendang, pTiltTendang, ballPositioningSpeed); // 0.15
            }
        }
    }
    // Localization ==============================================================================
    int kecepatanRobot = 25,  // cm
        finalGoalValue = 300; // jarak berhenti robot
    int jarakTiangKanan = 0,
        jarakTiangKiri = 0,
        goalDistanceValue = 0;
    double Timing = 0;
    int kalkulasiJarakGawang(int tiangKiri, int tiangKanan)
    {
        jarakTiangKiri = tiangKiri;
        jarakTiangKanan = tiangKanan;

        if (jarakTiangKiri > jarakTiangKanan)
        {
            if (jarakTiangKiri > 900)
            {
                goalDistanceValue = jarakTiangKanan;
            }
            else
            {
                goalDistanceValue = jarakTiangKiri;
            }
        }
        else
        {
            if (jarakTiangKanan > 900)
            {
                goalDistanceValue = jarakTiangKiri;
            }
            else
            {
                goalDistanceValue = jarakTiangKanan;
            }
        }

        if (jarakTiangKiri == -1 && jarakTiangKanan == -1)
        {
            Timing = 0;
        }
        else
        {
            Timing = (goalDistanceValue - finalGoalValue) / kecepatanRobot;
        }

        return Timing;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    double targetCoorY, sekarangCoorY, Ytar,
        targetCoorX, sekarangCoorX, Xtar,
        targetCoorR, sekarangCoorR, setPointRotateR1, setPointRotateR2,
        setfPointY,
        setRotatePointR,
        errorfX,
        errorfY,
        errorAngle,
        LyMove = 0, // Y
        LxMove = 0, // X
        LaMove = 0; // A
    int cL,         // count lock for atribut target
        cD,         // count peralihan dari rotate ke jalan
        cR,         // count peralihan dari break ke rotate
        cB,         // count untuk break ketika sudah didalam setpoint target
        cZ,
        cA,
        cV, cY,
        cN;
    bool donePosition = false,     // var untuk menandakan fungsi selesai
        kembaliMasuk = false,      // var untuk menandakan target diantara >90 && <-90
        awalMasuk = false,         // var untuk menandakan target diantara <90 && >-90
        cX = false,                // var untuk menandakan koordinat x sudah dalam range
        lockMidBack = false,       // var untuk mengunci posisi balik yang dituju -> tengah (main strategi)
        lockLeftBack = false,      // var untuk mengunci posisi balik yang dituju -> kiri (support strategi)
        lockRightBack = false,     // var untuk mengunci posisi balik yang dituju -> kanan (support strategi)
        balikTengah = false,       // var untuk lock posisi balik use koordinasi
        balikSampingKanan = false, // var untuk lock posisi balik use koordinasi
        balikSampingKiri = false;  // var untuk lock posisi balik use koordinasi
    //////////////////////////////////////////////////////////////////////////////////////////////

    int tama = 0;
    double Waktu = 0;
    void updateCoordinatFromVision()
    {
        if (RobotCoor_X >= -450 && RobotCoor_X <= 450 && RobotCoor_Y >= -300 && RobotCoor_Y <= 300)
        {
            if (RobotCoor_X != -1 && RobotCoor_Y != -1)
            {
                initialPos_X = RobotCoor_X;
                initialPos_Y = RobotCoor_Y;
                deltaPos_X = deltaPos_Y = 0;
            }
        }
    }

    bool backIn = false;
    int status[3], statusBackIn[3];
    int totalRobotStatus = 0, totalRobotStatusBackIn = 0;
    void koordinasiRobotBalik()
    { // robotStatus, jika 0 = kill jika 1 = run	//hari ini
        if (backIn == true)
        {
            if (robotNumber == 1)
            { // 3, 4, 5
                if (robot3Status == 1 && robot3BackIn == 0)
                {
                    status[0] = 1;
                    statusBackIn[0] = 0;
                }
                else if (robot3Status == 0 && robot3BackIn == 1)
                {
                    status[0] = 0;
                    statusBackIn[0] = 1;
                }
                else
                {
                    status[0] = 0;
                    statusBackIn[0] = 0;
                }

                if (robot4Status == 1 && robot4BackIn == 0)
                {
                    status[1] = 1;
                    statusBackIn[1] = 0;
                }
                else if (robot4Status == 0 && robot4BackIn == 1)
                {
                    status[1] = 0;
                    statusBackIn[1] = 1;
                }
                else
                {
                    status[1] = 0;
                    statusBackIn[1] = 0;
                }

                if (robot5Status == 1 && robot5BackIn == 0)
                {
                    status[2] = 1;
                    statusBackIn[2] = 0;
                }
                else if (robot5Status == 0 && robot5BackIn == 1)
                {
                    status[2] = 0;
                    statusBackIn[2] = 1;
                }
                else
                {
                    status[2] = 0;
                    statusBackIn[2] = 0;
                }
            }
            else if (robotNumber == 3)
            { // 1, 4, 5
                if (robot1Status == 1 && robot1BackIn == 0)
                {
                    status[0] = 1;
                    statusBackIn[0] = 0;
                }
                else if (robot1Status == 0 && robot1BackIn == 1)
                {
                    status[0] = 0;
                    statusBackIn[0] = 1;
                }
                else
                {
                    status[0] = 0;
                    statusBackIn[0] = 0;
                }

                if (robot4Status == 1 && robot4BackIn == 0)
                {
                    status[1] = 1;
                    statusBackIn[1] = 0;
                }
                else if (robot4Status == 0 && robot4BackIn == 1)
                {
                    status[1] = 0;
                    statusBackIn[1] = 1;
                }
                else
                {
                    status[1] = 0;
                    statusBackIn[1] = 0;
                }

                if (robot5Status == 1 && robot5BackIn == 0)
                {
                    status[2] = 1;
                    statusBackIn[2] = 0;
                }
                else if (robot5Status == 0 && robot5BackIn == 1)
                {
                    status[2] = 0;
                    statusBackIn[2] = 1;
                }
                else
                {
                    status[2] = 0;
                    statusBackIn[2] = 0;
                }
            }
            else if (robotNumber == 4)
            { // 1, 3, 5
                if (robot1Status == 1 && robot1BackIn == 0)
                {
                    status[0] = 1;
                    statusBackIn[0] = 0;
                }
                else if (robot1Status == 0 && robot1BackIn == 1)
                {
                    status[0] = 0;
                    statusBackIn[0] = 1;
                }
                else
                {
                    status[0] = 0;
                    statusBackIn[0] = 0;
                }

                if (robot3Status == 1 && robot3BackIn == 0)
                {
                    status[1] = 1;
                    statusBackIn[1] = 0;
                }
                else if (robot3Status == 0 && robot3BackIn == 1)
                {
                    status[1] = 0;
                    statusBackIn[1] = 1;
                }
                else
                {
                    status[1] = 0;
                    statusBackIn[1] = 0;
                }

                if (robot5Status == 1 && robot5BackIn == 0)
                {
                    status[2] = 1;
                    statusBackIn[2] = 0;
                }
                else if (robot5Status == 0 && robot5BackIn == 1)
                {
                    status[2] = 0;
                    statusBackIn[2] = 1;
                }
                else
                {
                    status[2] = 0;
                    statusBackIn[2] = 0;
                }
            }
            else if (robotNumber == 5)
            { // 1, 3, 4
                if (robot1Status == 1 && robot1BackIn == 0)
                {
                    status[0] = 1;
                    statusBackIn[0] = 0;
                }
                else if (robot1Status == 0 && robot1BackIn == 1)
                {
                    status[0] = 0;
                    statusBackIn[0] = 1;
                }
                else
                {
                    status[0] = 0;
                    statusBackIn[0] = 0;
                }

                if (robot3Status == 1 && robot3BackIn == 0)
                {
                    status[1] = 1;
                    statusBackIn[1] = 0;
                }
                else if (robot3Status == 0 && robot3BackIn == 1)
                {
                    status[1] = 0;
                    statusBackIn[1] = 1;
                }
                else
                {
                    status[1] = 0;
                    statusBackIn[1] = 0;
                }

                if (robot4Status == 1 && robot4BackIn == 0)
                {
                    status[2] = 1;
                    statusBackIn[2] = 0;
                }
                else if (robot4Status == 0 && robot4BackIn == 1)
                {
                    status[2] = 0;
                    statusBackIn[2] = 1;
                }
                else
                {
                    status[2] = 0;
                    statusBackIn[2] = 0;
                }
            }

            totalRobotStatus = status[0] + status[1] + status[2];
            totalRobotStatusBackIn = statusBackIn[0] + statusBackIn[1] + statusBackIn[2];

            if (totalRobotStatus == 0)
            {
                if (totalRobotStatusBackIn == 0)
                { // masuk tengah karna sendiri
                    balikSampingKiri = false;
                    balikSampingKanan = false;
                    balikTengah = true;
                }
                else if (totalRobotStatusBackIn != 0 && !balikTengah)
                {
                    if (totalRobotStatusBackIn < 3)
                    { // izin masuk diterima
                        // motion("9");
                        if (masukKiri)
                        {
                            balikSampingKiri = true;
                            balikSampingKanan = false;
                            balikTengah = false;
                        }
                        else if (masukKanan)
                        {
                            balikSampingKiri = false;
                            balikSampingKanan = true;
                            balikTengah = false;
                        }
                    }
                    else if (totalRobotStatusBackIn >= 3)
                    { // izin masuk ditolak
                        motion("0");
                    }
                }
            }
            else
            {
                if (totalRobotStatus < 3)
                { // izin masuk diterima
                    // motion("9");
                    if (masukKiri)
                    {
                        balikSampingKiri = true;
                        balikSampingKanan = false;
                        balikTengah = false;
                    }
                    else if (masukKanan)
                    {
                        balikSampingKiri = false;
                        balikSampingKanan = true;
                        balikTengah = false;
                    }
                }
                else if (totalRobotStatus >= 3)
                { // izin masuk ditolak
                    motion("0");
                }
            }
        }
        else if (backIn == false)
        {
            if (robotNumber == 1)
            {
                // printf("   this is Robot 1..........................\n");
                if (robotStatus == 1)
                {                       // run
                    balikTengah = true; // eksekusi
                    balikSampingKanan = false;
                    balikSampingKiri = false;
                    // printf("   Robot 1 Run...........................\n");
                }
                else
                { // kill
                    balikTengah = false;
                    balikSampingKanan = false;
                    balikSampingKiri = false;
                    // printf("   Robot 1 Kill..........................\n");
                }
            }
            else if (robotNumber == 3)
            {
                // printf("   this is Robot 3..........................\n");
                if (robotStatus == 1)
                { // run
                    if (robot1Status == 0)
                    { // ketika robot 1 tidak aktif
                        balikTengah = true;
                        balikSampingKanan = false;
                        balikSampingKiri = false;
                        // printf("   Robot 3 Run Balik Tengah..........................\n");
                    }
                    else if (robot1Status == 1)
                    { // ketika robot 1 aktif
                        balikTengah = false;
                        balikSampingKanan = true;
                        balikSampingKiri = false;
                        // printf("   Robot 3 Run Balik Samping Kanan..........................\n");
                    }
                }
                else
                { // kill
                    balikTengah = false;
                    balikSampingKanan = false;
                    balikSampingKiri = false;
                    // printf("   Robot 3 Kill..........................\n");
                }
            }
            else if (robotNumber == 4)
            {
                // printf("   this is Robot 4..........................\n");
                if (robotStatus == 1)
                { // run
                    if (robot1Status == 0)
                    { // ketika robot 1 tidak aktif
                        if (robot3Status == 0)
                        { // ketika robot 3 tidak aktif
                            balikTengah = true;
                            balikSampingKanan = false;
                            balikSampingKiri = false;
                            // printf("   Robot 4 Run Balik Tengah..........................\n");
                        }
                        else if (robot3Status == 1)
                        { // ketika robot 3 aktif
                            balikTengah = false;
                            balikSampingKanan = true;
                            balikSampingKiri = false;
                            // printf("   Robot 4 Run Balik Samping Kanan..........................\n");
                        }
                    }
                    else if (robot1Status == 1)
                    { // ketika robot 1 aktif
                        if (robot3Status == 0)
                        { // ketika robot 3 tidak aktif
                            balikTengah = false;
                            balikSampingKanan = true;
                            balikSampingKiri = false;
                            // printf("   Robot 4 Run Balik Samping Kanan..........................\n");
                        }
                        else if (robot3Status == 1)
                        { // ketika robot 3 aktif
                            balikTengah = false;
                            balikSampingKanan = false;
                            balikSampingKiri = true;
                            // printf("   Robot 4 Run Balik Samping Kiri..........................\n");
                        }
                    }
                }
                else
                { // kill
                    balikTengah = false;
                    balikSampingKanan = false;
                    balikSampingKiri = false;
                    // printf("   Robot 4 Kill..........................\n");
                }
            }
            else if (robotNumber == 5)
            {
                // printf("   this is Robot 5..........................\n");
                if (robotStatus == 1)
                { // Run
                    if (robot1Status == 0)
                    { // ketika robot 1 tidak aktif
                        if (robot3Status == 0 || robot4Status == 0)
                        { // ketika robot 3 tidak aktif atau robot 4 tidak aktif
                            balikTengah = true;
                            balikSampingKanan = false;
                            balikSampingKiri = false;
                            // printf("   Robot 5 Run Balik Samping Tengah..........................\n");
                        }
                        else if (robot3Status == 1 || robot4Status == 1)
                        { // ketiak robot 3 aktif atau robot 4 akti
                            balikTengah = false;
                            balikSampingKanan = true;
                            balikSampingKiri = false;
                            // printf("   Robot 5 Run Balik Samping Kanan..........................\n");
                        }
                    }
                    else if (robot1Status == 1)
                    { // ketika robot 1 aktif
                        if (robot3Status == 0 || robot4Status == 0)
                        { // ketika robot 3 tidak aktif atau robot 4 tidak aktif
                            balikTengah = false;
                            balikSampingKanan = true;
                            balikSampingKiri = false;
                            // printf("   Robot 5 Run Balik Samping Kanan..........................\n");
                        }
                        else if (robot3Status == 1 || robot4Status == 1)
                        { // ketika robot 3 aktif atau robot 4 aktif
                            balikTengah = false;
                            balikSampingKanan = false;
                            balikSampingKiri = true;
                            // printf("   Robot 5 Run Balik Samping Kiri..........................\n");
                        }
                    }
                }
                else
                {
                    balikTengah = false;
                    balikSampingKanan = false;
                    balikSampingKiri = false;
                    // printf("   Robot 5 Kill..........................\n");
                }
            }
        }
    }

    void moveLokalisasi(double mode, double Xtarget, double Ytarget)
    {
        /*Robot berjalan berdasarkan koordinat yang di imput*/
        Xtar = Xtarget;
        Ytar = Ytarget;
        targetCoorX = robotPos_X - Xtarget; //-240--240	= 0
        targetCoorY = robotPos_Y - Ytarget; //-320--0	= -320
        targetCoorR = sqrt((targetCoorX * targetCoorX) + (targetCoorY * targetCoorY));
        if (robotPos_X > Xtarget)
        { // Target ada dibelakang
            if (robotPos_Y > Ytarget)
            { // saat ini sebelah kanan
                setRotatePointR = -180 + asin(targetCoorY / targetCoorR) * (180 / PI);
            }
            else if (robotPos_Y < Ytarget)
            { // saat ini sebelah kiri
                setRotatePointR = 180 + asin(targetCoorY / targetCoorR) * (180 / PI);
            }
        }
        else if (robotPos_X < Xtarget)
        { // Target ada didepan
            if (robotPos_Y < Ytarget)
            { // saat ini sebelah kiri
                setRotatePointR = 0 - asin(targetCoorY / targetCoorR) * (180 / PI);
            }
            else if (robotPos_Y > Ytarget)
            { // saat ini sebalah kanan
                setRotatePointR = 0 - asin(targetCoorY / targetCoorR) * (180 / PI);
            }
        }

        if (robotPos_X < (Xtarget + 15) && robotPos_X > (Xtarget - 15))
        { // Koreksi koordinatX
            cX = true;
        }
        else
        {
            cB = 0;
        }
        if (cX)
        {
            if (mode == 0)
            { // Koreksi koordinat Y Jika koordinat X sudah masuk range
                if (robotPos_Y < (Ytarget + 15) && robotPos_Y > (Ytarget - 15))
                {
                    cB++;
                }
                else
                {
                    if (robotPos_Y > (Ytarget + 15))
                    {
                        if (cY > 10)
                        {
                            if (posRotateNew && cX)
                            {
                                jalanDirection(kejarMax, 0.0, -90);
                            }
                            else
                            {
                                rotateBodyImuNew(-90);
                            }
                        }
                        else
                        {
                            Walk(0.0, 0.0, 0.0);
                            cY++;
                        }
                    }
                    else if (robotPos_Y < (Ytarget - 15))
                    {
                        if (cY > 10)
                        {
                            if (posRotateNew && cX)
                            {
                                jalanDirection(kejarMax, 0.0, 90);
                            }
                            else
                            {
                                rotateBodyImuNew(90);
                            }
                        }
                        else
                        {
                            Walk(0.0, 0.0, 0.0);
                            cY++;
                        }
                    }
                }
            }
            else if (mode == 1)
            { // tanpa koreksi koordinat Y
                cB++;
            }
        }

        if (cB >= 1)
        {
            donePosition = true;
            posRotateNew = false;
            LxMove = 0.0;
        }
        else
        {
            if (!cX)
            {
                LxMove = kejarMax;
                jalanDirection(LxMove, 0.0, setRotatePointR); // Jika bisa baca gawang
            }
        }
    }

    void moveBackLokalisasi(double mode, double Xtarget, double Ytarget)
    {
        /*Robot berjalan berdasarkan koordinat yang di imput*/
        Xtar = Xtarget;
        Ytar = Ytarget;
        targetCoorX = robotPos_X - Xtarget; //-240--240	= 0
        targetCoorY = robotPos_Y - Ytarget; //-320--0	= -320
        targetCoorR = sqrt((targetCoorX * targetCoorX) + (targetCoorY * targetCoorY));
        if (robotPos_X > Xtarget)
        { // Target ada dibelakang
            if (robotPos_Y > Ytarget)
            { // saat ini sebelah kanan
                setRotatePointR = -180 + asin(targetCoorY / targetCoorR) * (180 / PI);
            }
            else if (robotPos_Y < Ytarget)
            { // saat ini sebelah kiri
                setRotatePointR = 180 + asin(targetCoorY / targetCoorR) * (180 / PI);
            }
        }
        else if (robotPos_X < Xtarget)
        { // Target ada didepan
            if (robotPos_Y < Ytarget)
            { // saat ini sebelah kiri
                setRotatePointR = 0 - asin(targetCoorY / targetCoorR) * (180 / PI);
            }
            else if (robotPos_Y > Ytarget)
            { // saat ini sebalah kanan
                setRotatePointR = 0 - asin(targetCoorY / targetCoorR) * (180 / PI);
            }
        }

        if (robotPos_X < (Xtarget + 15) && robotPos_X > (Xtarget - 15))
        { // Koreksi koordinatX
            cX = true;
        }
        else
        {
            cB = 0;
        }
        if (cX)
        {
            if (mode == 0)
            { // Koreksi koordinat Y Jika koordinat X sudah masuk range
                if (robotPos_Y < (Ytarget + 15) && robotPos_Y > (Ytarget - 15))
                {
                    cB++;
                }
                else
                {
                    if (robotPos_Y > (Ytarget + 15))
                    {
                        if (cY > 10)
                        {
                            if (posRotateNew && cX)
                            {
                                jalanDirection(kejarMax, 0.0, -90);
                            }
                            else
                            {
                                rotateBodyImuNew(-90);
                            }
                        }
                        else
                        {
                            Walk(0.0, 0.0, 0.0);
                            cY++;
                        }
                    }
                    else if (robotPos_Y < (Ytarget - 15))
                    {
                        if (cY > 10)
                        {
                            if (posRotateNew && cX)
                            {
                                jalanDirection(kejarMax, 0.0, 90);
                            }
                            else
                            {
                                rotateBodyImuNew(90);
                            }
                        }
                        else
                        {
                            Walk(0.0, 0.0, 0.0);
                            cY++;
                        }
                    }
                }
            }
            else if (mode == 1)
            { // tanpa koreksi koordinat Y
                cB++;
            }
        }

        if (cB >= 1)
        {
            donePosition = true;
            posRotateNew = false;
            LxMove = 0.0;
        }
        else
        {
            if (!cX)
            {
                LxMove = kejarMax;
                // jalanDirection(LxMove,0.0,setRotatePointR);	//Jika bisa baca gawang
                jalanDirection(LxMove, 0.0, 180); // Jika tidak bisa baca gawang
            }
        }
    }

    // New Ability 2020 use Grid in Field
    void gridLocalization()
    { // Konvert koordinat posisi robot (x,y) jadi Grid posisi robot
        int tempGridX, tempGridY, GridX, GridY;
        /*
        if (robotPos_X >= -450 && robotPos_X <= 450 && robotPos_Y >= -300 && robotPos_Y <= 300) {
            tempGridX = ((robotPos_X+450)/100)+1;
            tempGridY = ((robotPos_Y+300)/100)+1;

            if (tempGridX >= 9) {tempGridX = 9;}
            else if (tempGridX <= 0) {tempGridX = 0;}
            if (tempGridY >= 6) {tempGridY = 6;}
            else if (tempGridY <= 0) {tempGridY = 0;}

            if (tempGridX > 0 && tempGridX <=9 && tempGridY > 0 && tempGridY <=6){
                Grid = tempGridY + (6*(tempGridX-1));
                //printf("%d\n",Grid);
                if (Grid <= 1) { Grid = 1;}
                else if (Grid >= 54) { Grid = 54;}
            } else {
                Grid  = 88;
            }
        } else {
            Grid = 88;
        }
        */
        GridX = robotPos_X;
        GridY = robotPos_Y;

        if (GridX >= 450)
        {
            GridX = 450;
        }
        else if (GridX <= -450)
        {
            GridX = -450;
        }
        if (GridY >= 300)
        {
            GridY = 300;
        }
        else if (GridY <= -300)
        {
            GridY = -300;
        }

        tempGridX = ((GridX + 450) / 100) + 1;
        tempGridY = ((GridY + 300) / 100) + 1;

        if (tempGridX >= 9)
        {
            tempGridX = 9;
        }
        else if (tempGridX <= 1)
        {
            tempGridX = 1;
        }
        if (tempGridY >= 6)
        {
            tempGridY = 6;
        }
        else if (tempGridY <= 1)
        {
            tempGridY = 1;
        }

        Grid = tempGridY + (6 * (tempGridX - 1));
        if (Grid <= 1)
        {
            Grid = 1;
        }
        else if (Grid >= 54)
        {
            Grid = 54;
        }
    }

    void gridBall()
    { // Konvert koordinat posisi bola (x,y) jadi Grid posisi bola
        int tempX, tempY;
        // if (Ball_X != 0 && Ball_Y != 0) {
        if (Ball_D > 0 && Ball_D <= 900)
        {
            if (BallCoor_X != 0 || BallCoor_Y != 0)
            {
                tempX = ((BallCoor_X + 450) / 100) + 1;
                tempY = ((BallCoor_Y + 300) / 100) + 1;

                if (tempX >= 9)
                {
                    tempX = 9;
                }
                else if (tempX <= 0)
                {
                    tempX = 0;
                }
                if (tempY >= 6)
                {
                    tempY = 6;
                }
                else if (tempY <= 0)
                {
                    tempY = 0;
                }

                if (tempX > 0 && tempX <= 9 && tempY > 0 && tempY <= 6)
                {
                    GridBall = tempY + (6 * (tempX - 1));
                    // printf("%d\n",GridBall);
                    if (GridBall <= 1)
                    {
                        GridBall = 1;
                    }
                    else if (GridBall >= 54)
                    {
                        GridBall = 54;
                    }
                }
                else
                {
                    GridBall = 88;
                }
            }
            else
            {
                GridBall = 88;
            }
        }
        else
        {
            GridBall = 88;
        }
    }

    int lastGrid;
    void saveGrid()
    { // Save nilai grid
        lastGrid = Grid;
    }

    int convertGridX(int valueGrid, int valueOffSetX)
    { // Konvert grid posisi robot jadi nilai koordinat x
        int tempCoorX, tempGridtoX;

        if (valueGrid % 6 == 0)
        {
            // printf("POPO\n");
            tempGridtoX = valueGrid / 6;
        }
        else
        {
            // printf("PIPI\n");
            tempGridtoX = (valueGrid / 6) + 1;
        }

        if (tempGridtoX <= 1)
        {
            tempGridtoX = 1;
        }
        else if (tempGridtoX >= 9)
        {
            tempGridtoX = 9;
        }

        tempCoorX = (((tempGridtoX * 100) - 500) + valueOffSetX);

        return tempCoorX;
    }

    int convertGridY(int valueGrid, int valueOffSetY)
    { // Konvert grid posisi robot jadi nilai koordinat y
        int tempCoorY, tempGridtoY;
        if (valueGrid % 6 == 0)
        {
            // printf("POPO\n");
            tempGridtoY = 6;
        }
        else
        {
            // printf("PIPI\n");
            tempGridtoY = (valueGrid - ((valueGrid / 6) * 6));
        }

        if (tempGridtoY <= 1)
        {
            tempGridtoY = 1;
        }
        else if (tempGridtoY >= 6)
        {
            tempGridtoY = 6;
        }

        tempCoorY = (((tempGridtoY * 100) - 350) + valueOffSetY);

        return tempCoorY;
    }

    bool doneHeaded = false;
    void headGrid(int valueGrid, int valueOffSetX, int valueOffSetY)
    { // menghadap posisi Grid yang tentukan
        double x, y, r, rotate;
        /*Robot berjalan berdasarkan koordinat yang di imput*/
        x = robotPos_X - convertGridX(valueGrid, valueOffSetX); //-240--240	= 0
        y = robotPos_Y - convertGridY(valueGrid, valueOffSetY); //-320--0	= -320
        r = sqrt((x * x) + (y * y));
        // printf("ADA APA\n");
        if (robotPos_X >= convertGridX(valueGrid, valueOffSetX))
        { // Target ada dibelakang
            // printf("MASUK HAHA\n");
            if (robotPos_Y >= convertGridY(valueGrid, valueOffSetY))
            { // saat ini sebelah kanan
                rotate = -180 + asin(y / r) * (180 / PI);
            }
            else if (robotPos_Y < convertGridY(valueGrid, valueOffSetY))
            { // saat ini sebelah kiri
                rotate = 180 + asin(y / r) * (180 / PI);
            }
        }
        else if (robotPos_X < convertGridX(valueGrid, valueOffSetX))
        { // Target ada didepan
            // printf("MASUK HIHI\n");
            if (robotPos_Y < convertGridY(valueGrid, valueOffSetY))
            { // saat ini sebelah kiri
                // printf("MASUK HUHU\n");
                rotate = 0 - asin(y / r) * (180 / PI);
                // printf("%.2lf\n",valueRotateBody);
            }
            else if (robotPos_Y >= convertGridY(valueGrid, valueOffSetY))
            { // saat ini sebalah kanan
                // printf("MASUK HOHO\n");
                rotate = 0 - asin(y / r) * (180 / PI);
            }
        }
        if (posRotateNew)
        {
            doneHeaded = true;
            posRotateNew = false;
        }
        else
        {
            rotateBodyImuNew(rotate);
        }
    }

    bool doneMoved = false,
         setGrid1 = false,
         setGrid2 = false;

    int countMoveGrid1 = 0, // count walk ditempat
        countMoveGrid2 = 0, // count rotate
        countMoveGrid3 = 0; // count walk x,y

    double rotateMoveGrid = 0;

    void moveGrid(int valueGrid, int valueOffSetX, int valueOffSetY)
    { // Bergerak menuju grid yang ditentukan
        double c, s, sn, x, y, r, rotate, speedX, speedY, speedrX, speedrY, nilaiSudut;

        c = cos(angle);
        s = sin(angle);
        sn = sin(angle) * -1;

        if (angle < 0)
        {
            nilaiSudut = angle + 360;
        }
        else
        {
            nilaiSudut = angle;
        }
        // printf("Nilai Sudut = %.2lf\n", nilaiSudut);
        x = robotPos_X - convertGridX(valueGrid, valueOffSetX); //-240--240	= 0
        y = robotPos_Y - convertGridY(valueGrid, valueOffSetY); //-320--0	= -320
        r = sqrt((x * x) + (y * y));
        // printf("NILAI R = %.2lf\n",r);
        // printf("ADA APA\n");
        if (robotPos_X >= convertGridX(valueGrid, valueOffSetX))
        { // Target ada dibelakang
            // printf("MASUK HAHA\n");
            if (robotPos_Y >= convertGridY(valueGrid, valueOffSetY))
            { // saat ini sebelah kanan
                rotate = -180 + asin(y / r) * (180 / PI);
            }
            else if (robotPos_Y < convertGridY(valueGrid, valueOffSetY))
            { // saat ini sebelah kiri
                rotate = 180 + asin(y / r) * (180 / PI);
            }
        }
        else if (robotPos_X < convertGridX(valueGrid, valueOffSetX))
        { // Target ada didepan
            // printf("MASUK HIHI\n");
            if (robotPos_Y < convertGridY(valueGrid, valueOffSetY))
            { // saat ini sebelah kiri
                // printf("MASUK HUHU\n");
                rotate = 0 - asin(y / r) * (180 / PI);
                // printf("%.2lf\n",valueRotateBody);
            }
            else if (robotPos_Y >= convertGridY(valueGrid, valueOffSetY))
            { // saat ini sebalah kanan
                // printf("MASUK HOHO\n");
                rotate = 0 - asin(y / r) * (180 / PI);
            }
        }
        rotateMoveGrid = rotate;
        if (robotPos_X >= (convertGridX(valueGrid, valueOffSetX) - 15) && robotPos_X < (convertGridX(valueGrid, valueOffSetX) + 15) &&
            robotPos_Y >= (convertGridY(valueGrid, valueOffSetY) - 15) && robotPos_Y < (convertGridY(valueGrid, valueOffSetY) + 15))
        {
            countMoveGrid1 =
                countMoveGrid2 =
                    countMoveGrid3 = 0;

            posRotateNew =
                setGrid1 =
                    setGrid2 = false;
            doneMoved = true;
        }
        else
        {
            if (countMoveGrid1 >= 5)
            {
                // RY < TY => RY = -30 TY = -25 DY = -30
                if (r < 30)
                {
                    // printf("TIME TO SHOWWWWWW>>>>>> \n");
                    if (countMoveGrid2 >= 5)
                    {
                        if (nilaiSudut > 270 || nilaiSudut <= 90)
                        {
                            // printf("MASUK 0 0 0 0\n");
                            if (posRotateNew)
                            {
                                // printf("SELESAI ROTATE 0\n");
                                if (countMoveGrid3 >= 5)
                                {
                                    if (!setGrid1 && !setGrid2)
                                    {
                                        if (robotPos_X >= convertGridX(valueGrid, valueOffSetX))
                                        {
                                            setGrid1 = true; // RX = -300 TX = -400 DX = -300 -(-400) = 100
                                                             // RX = 30 TX = -400 DX = 30 -(-400) = 430
                                        }
                                        else if (robotPos_X < convertGridX(valueGrid, valueOffSetX))
                                        {
                                            setGrid2 = true; // RX = 300 TX = 400 DX = 300 - 400 = -100
                                                             // RX = -30 TX = -25 DX = -30 -(-25) = -5
                                        }
                                    }
                                    else
                                    {
                                        if (setGrid1)
                                        {
                                            speedX = x * 0.02;
                                            // printf("SET GRID 1\n");
                                        }
                                        else if (setGrid2)
                                        {
                                            speedX = x * -0.02;
                                            // printf("SET GRID 2\n");
                                        }
                                    }

                                    speedY = y * 0.02;

                                    if (speedX >= 0.08)
                                    {
                                        speedX = 0.08;
                                    }
                                    else if (speedX <= -0.03)
                                    {
                                        speedX = -0.03;
                                    }
                                    if (speedY >= 0.04)
                                    {
                                        speedY = 0.04;
                                    }
                                    else if (speedY <= -0.04)
                                    {
                                        speedY = -0.04;
                                    }

                                    Walk(speedX, speedY, 0.0);
                                }
                                else
                                {
                                    Walk(0.0, 0.0, 0.0);
                                    countMoveGrid3++;
                                }
                            }
                            else
                            {
                                rotateBodyImuNew(0);
                            }
                        }
                        else if (nilaiSudut > 90 || nilaiSudut <= 270)
                        {
                            // printf("MASUK 180 180 180 180\n");
                            if (posRotateNew)
                            {
                                // printf("SELESAI ROTATE 0\n");
                                if (countMoveGrid3 >= 5)
                                {
                                    if (!setGrid1 && !setGrid2)
                                    {
                                        if (robotPos_X >= convertGridX(valueGrid, valueOffSetX))
                                        {
                                            setGrid1 = true; // RX = -300 TX = -400 DX = -300 -(-400) = 100
                                                             // RX = 30 TX = -400 DX = 30 -(-400) = 430
                                        }
                                        else if (robotPos_X < convertGridX(valueGrid, valueOffSetX))
                                        {
                                            setGrid2 = true; // RX = 300 TX = 400 DX = 300 - 400 = -100
                                                             // RX = -30 TX = -25 DX = -30 -(-25) = -5
                                        }
                                    }
                                    else
                                    {
                                        if (setGrid1)
                                        {
                                            speedX = x * 0.02;
                                            // printf("SET GRID 1\n");
                                        }
                                        else if (setGrid2)
                                        {
                                            speedX = x * -0.02;
                                            // printf("SET GRID 2\n");
                                        }
                                    }
                                    speedY = y * 0.02;

                                    if (speedX >= 0.08)
                                    {
                                        speedX = 0.08;
                                    }
                                    else if (speedX <= -0.03)
                                    {
                                        speedX = -0.03;
                                    }
                                    if (speedY >= 0.04)
                                    {
                                        speedY = 0.04;
                                    }
                                    else if (speedY <= -0.04)
                                    {
                                        speedY = -0.04;
                                    }

                                    Walk(speedX, -speedY, 0.0);
                                }
                                else
                                {
                                    Walk(0.0, 0.0, 0.0);
                                    countMoveGrid3++;
                                }
                            }
                            else
                            {
                                rotateBodyImuNew(180);
                            }
                        }
                    }
                    else
                    {
                        posRotateNew = false;
                        Walk(0.0, 0.0, 0.0);
                        countMoveGrid2++;
                    }
                }
                else
                {
                    setGrid1 = false;
                    setGrid2 = false;
                    posRotateNew = false;
                    countMoveGrid2 = 0;
                    countMoveGrid3 = 0;
                    jalanDirection(kejarMax, 0.0, rotate);
                }
            }
            else
            {
                Walk(0.0, 0.0, 0.0);
                countMoveGrid1++;
            }
        }
    }

    bool inversValue = false;
    void localization()
    {
        if (reset > 5)
        {
            // lock initial pos
            // if (lastSecRemaining == 0) { //jika data game controller tidak terbaca
            if (Remaining == 0)
            { // jika data game controller tidak terbaca
                motion("0");
                resetAllVariable();
                predictGoal(angle, posTiltGoal);
                backPosition = false;
                //} else if (lastSecRemaining == 600) { //masuk lapangan pertama kali mulai permainan
            }
            else if ((Remaining == 600 && SecondaryState == 0) || (Remaining == 300 && SecondaryState == 2))
            { // masuk lapangan pertama kali mulai permainan
                if (awalMasuk)
                { // ketika sudah sampai posisi awal masuk
                    // refreshMoveLokalisasi();
                    motion("0");
                    refreshMoveGrid();
                    if (goalLost(20))
                    {
                        panSearchBall(posTiltLocal);
                    }
                    else
                    {
                        trackGoal();
                        if (useLocalization && useUpdateCoordinate)
                        {
                            updateCoordinatFromVision();
                        }
                    }
                }
                else
                { // ketika belum sampai posisi masuk
                    if (doneMoved)
                    {
                        predictGoal(0.0, posTiltGoal);
                        if (cR >= 10)
                        {
                            if (strategyCM < 3)
                            {
                                if (posRotateNew)
                                {
                                    awalMasuk = true;
                                }
                                else
                                {
                                    rotateBodyImuNew(0);
                                }
                            }
                            else if (strategyCM == 3)
                            {
                                if (posRotateNew)
                                {
                                    awalMasuk = true;
                                }
                                else
                                {
                                    rotateBodyImuNew(ArchSinTeng);
                                }
                            }
                        }
                        else
                        {
                            posRotateNew = false;
                            Walk(0.0, 0.0, 0.0);
                            cR++;
                        }
                    }
                    else
                    {
                        motion("9");
                        if (walkTot >= 6)
                        {
                            searchBallRectang(-1.6, -1.6, -0.8, 1.6);
                            moveGrid(initGrid, offsetX, offsetY);
                        }
                        else
                        { // sini dulu, sampai counting jalan terpenuhi
                            predictGoal(angle, -1.6);
                            if (count > 5)
                            { // counting biasa terpenuhi
                                Walk(kejarMax, 0.0, 0.0);
                            }
                            else
                            { /// sini, counting dulu
                                count++;
                                Walk(0.0, 0.0, 0.0);
                            }
                        }
                    }
                }
                backPosition = false;
                //} else if (lastSecRemaining != 600) { //kembali ke posisi setelah ada goal/dropball
            }
            else if ((Remaining != 600 && SecondaryState == 0) || (Remaining != 300 && SecondaryState == 2))
            { // kembali ke posisi setelah ada goal/dropball
                // printf("\n  >>>>>>>> balik ke belakang, goal/dropball <<<<<<<< \n");
                if (kembaliMasuk)
                {
                    motion("0");
                    // refreshMoveLokalisasi();
                    count = 0;
                    refreshMoveGrid();
                    if (goalLost(20))
                    {
                        panSearchBall(posTiltLocal);
                    }
                    else
                    {
                        trackGoal();
                        if (useLocalization && useUpdateCoordinate)
                        {
                            updateCoordinatFromVision();
                        }
                    }
                }
                else
                {
                    if (useCoordination)
                    { // kondisi balik ketika menggunakan useCoordination
                        if (doneMoved)
                        {
                            inversValue = false;
                            predictGoal(0, posTiltGoal);
                            if (cR >= 10)
                            {
                                if (balikTengah)
                                {
                                    if (posRotateNew)
                                    {
                                        kembaliMasuk = true;
                                        // refreshMoveGrid();
                                    }
                                    else
                                    {
                                        rotateBodyImuNew(0);
                                    }
                                }
                                else if (balikSampingKanan || balikSampingKiri)
                                {
                                    if (posRotateNew)
                                    {
                                        kembaliMasuk = true;
                                        // refreshMoveGrid();
                                    }
                                    else
                                    {
                                        rotateBodyImuNew(ArchSinTeng);
                                    }
                                }
                            }
                            else
                            {
                                posRotateNew = false;
                                Walk(0.0, 0.0, 0.0);
                                cR++;
                            }
                        }
                        else
                        {
                            koordinasiRobotBalik();
                            if (count > 5)
                            {
                                inversValue = true;
                                if (backIn)
                                {
                                    // searchBallRectang(-1.5, -1.6, -0.8, 1.6);
                                }
                                else
                                {
                                    /*
                                    if (rotateMoveGrid >= -90 || rotateMoveGrid <= 90) {	//Arah pandang ke gawang lawan
                                        predictGoal(ArchSinEnemy, posTiltGoal);
                                    } else {						//Arah pandang ke gawang team
                                        predictGoalTeam(ArchSinTeam, posTiltGoal);
                                    }*/
                                    if (goalLost(20))
                                    {
                                        panSearchBall(posTiltLocal);
                                    }
                                    else
                                    {
                                        trackGoal();
                                        if (useLocalization && useUpdateCoordinate)
                                        {
                                            updateCoordinatFromVision();
                                        }
                                    }
                                }

                                /* if (!readyMoveGrid)
                                {
                                    if (goalLost(20))
                                    {
                                        tracked = false;
                                    } else {
                                        tracked = true;
                                    }

                                    if (tracked)
                                    {
                                        trackGoal();
                                        if (robotDirection && headPan >= -0.4 && headPan <= 0.4)
                                        {
                                            if (angle < 0)
                                            {
                                                koorRobotX = -40.0;
                                                koorRobotY = 80.0;
                                            } else {
                                                koorRobotX = -40.0;
                                                koorRobotY = -80.0;
                                            }
                                            loadKoordinatRobot();
                                            readyMoveGrid = true;
                                                // motion("0");
                                        } else
                                        {
                                            motion("9");
                                            if (headTilt >= cAktif && headPan >= -0.4 && headPan <= 0.4)
                                            {
                                                Imu(179, cSekarang);
                                            } else {
                                                followGoal(0.08,0.0,0);
                                            }
                                            readyMoveGrid = false;
                                        }
                                        printf("tracked\n");
                                    } else {
                                        motion("9");
                                        tiltSearchBall(0.0);
                                        jalanDirection(0.08, 0.0, 179);
                                        printf("search goal\n");
                                    }
                                }
     */
                                if (KickOff == barelang_color || KickOff == dropball)
                                { // Attack
                                    printf("  Our robot kick off !!!\n");
                                    if (balikTengah)
                                    {
                                        motion("9");
                                        moveGrid(21, 0, 50);
                                        balikSampingKiri = false;
                                        balikSampingKanan = false;
                                    }
                                    else if (balikSampingKanan)
                                    {
                                        motion("9");
                                        moveGrid(23, -50, 0);
                                        balikSampingKiri = false;
                                        balikTengah = false;
                                    }
                                    else if (balikSampingKiri)
                                    {
                                        motion("9");
                                        moveGrid(23, -50, 0);
                                        balikSampingKanan = false;
                                        balikTengah = false;
                                    }
                                    else
                                    {
                                        motion("0");
                                        predictGoal(angle, posTiltGoal);
                                    }
                                }
                                else
                                { // Defense
                                    printf("   Our robot defend !!!\n");
                                    if (balikTengah)
                                    {
                                        motion("9");
                                        moveGrid(21, -50, 50);
                                        balikSampingKiri = false;
                                        balikSampingKanan = false;
                                    }
                                    else if (balikSampingKanan)
                                    {
                                        motion("9");
                                        moveGrid(17, -50, 0);
                                        balikSampingKiri = false;
                                        balikTengah = false;
                                    }
                                    else if (balikSampingKiri)
                                    {
                                        motion("9");
                                        moveGrid(14, -50, 0);
                                        balikSampingKanan = false;
                                        balikTengah = false;
                                    }
                                    else
                                    {
                                        motion("0");
                                        predictGoal(angle, posTiltGoal);
                                    }
                                }
                            }
                            else
                            {
                                Walk(0.0, 0.0, 0.0);
                                count++;
                            }
                        }
                    }
                    else
                    { // tanpa koordinasi
                        if (doneMoved)
                        {
                            inversValue = false;
                            predictGoal(0, posTiltGoal);
                            if (cR >= 10)
                            {
                                if (strategyCM < 3)
                                {
                                    if (posRotateNew)
                                    {
                                        kembaliMasuk = true;
                                    }
                                    else
                                    {
                                        rotateBodyImuNew(0);
                                    }
                                }
                                else if (strategyCM == 3)
                                {
                                    if (posRotateNew)
                                    {
                                        kembaliMasuk = true;
                                    }
                                    else
                                    {
                                        rotateBodyImuNew(ArchSinTeng);
                                    }
                                }
                            }
                            else
                            {
                                posRotateNew = false;
                                Walk(0.0, 0.0, 0.0);
                                cR++;
                            }
                        }
                        else
                        {
                            if (count >= 5)
                            {
                                inversValue = true;
                                motion("9");

                                if (rotateMoveGrid >= -90 || rotateMoveGrid <= 90)
                                { // Arah pandang ke gawang lawan
                                    predictGoal(ArchSinEnemy, posTiltGoal);
                                }
                                else
                                { // Arah pandang ke gawang team
                                    predictGoalTeam(ArchSinTeam, posTiltGoal);
                                }

                                if (useLocalization && useUpdateCoordinate)
                                {
                                    updateCoordinatFromVision();
                                }

                                if (KickOff == barelang_color || KickOff == dropball)
                                { // Attack
                                    if (strategyCM < 3)
                                    { // balik depan kondisi kick off
                                        moveGrid(21, 0, 50);
                                    }
                                    else if (strategyCM == 3)
                                    { // balik samping kondisi kick off
                                        if (!lockLeftBack && !lockRightBack)
                                        { // penentuan kiri atau kanan
                                            if (robotPos_Y >= 0)
                                            {
                                                lockRightBack = true;
                                            }
                                            else if (robotPos_Y < 0)
                                            {
                                                lockLeftBack = true;
                                            }
                                        }
                                        else
                                        {
                                            if (lockLeftBack)
                                            { // balik kiri
                                                moveGrid(20, -50, 0);
                                            }
                                            else if (lockRightBack)
                                            { // balik kanan
                                                moveGrid(23, -50, 0);
                                            }
                                        }
                                    }
                                }
                                else
                                { // Defense
                                    if (strategyCM < 3)
                                    { // balik depan kondisi kick off
                                        moveGrid(21, -50, 50);
                                    }
                                    else if (strategyCM == 3)
                                    { // balik samping kondisi kick off
                                        if (!lockLeftBack && !lockRightBack)
                                        { // penentuan kiri atau kanan
                                            if (robotPos_Y >= 0)
                                            {
                                                lockRightBack = true;
                                            }
                                            else if (robotPos_Y < 0)
                                            {
                                                lockLeftBack = true;
                                            }
                                        }
                                        else
                                        {
                                            if (lockLeftBack)
                                            { // balik kiri
                                                moveGrid(14, -50, 0);
                                            }
                                            else if (lockRightBack)
                                            { // balik kanan
                                                moveGrid(17, -50, 0);
                                            }
                                        }
                                    }
                                }
                            }
                            else
                            {
                                inversValue = false;
                                Walk(0.0, 0.0, 0.0);
                                count++;
                            }
                        }
                    }
                }
                backPosition = true;
            }
        }
        else
        {
            cR = 0;
            refreshMoveLokalisasi();
            refreshMoveGrid();
            posRotate = false;
            lockMidBack = false;
            lockLeftBack = false;
            lockRightBack = false;
            kembaliMasuk = false;
            awalMasuk = false;
            reset++;
        }
    }

    int cset = 0;
    void robotPositioning(int mode)
    {
        if (Remaining == 0)
        {
            motion("0");
        }
        else if ((Remaining == 600 && SecondaryState == 0) || (Remaining == 300 && SecondaryState == 2))
        {
            printf("cset, timer, mode = %d, %d, %d\n");
            if (mode == 0) // attack depan
            {
                printf("masuk depan\n");
                headMove(0.0, -1.2);
                if (cset > 5)
                {
                    cekWaktu(19);
                    if (timer)
                    {
                        motion("0");
                    }
                    else if (second > 18 || second < 1)
                    {
                        motion("9");
                        jalanDirection(0.0, 0.0, 0.0);
                    }
                    else
                    {
                        jalanDirection(0.06, 0.0, 0);
                    }
                }
                else
                {
                    setWaktu();
                    cset++;
                }
            }
            else if (mode == 1) // belakang
            {
                printf("masuk belakang\n");
                headMove(0.0, -1.5);
                if (cset > 5)
                {
                    cekWaktu(11);
                    if (timer)
                    {
                        motion("0");
                    }
                    else if (second > 10 || second < 1)
                    {
                        motion("9");
                        jalanDirection(0.0, 0.0, 0.0);
                    }
                    else
                    {
                        jalanDirection(0.06, 0.0, 0.0);
                    }
                }
                else
                {
                    setWaktu();
                    cset++;
                }
            }
            else if (mode == 2) // depan defense
            {
                printf(" depan defense\n");
                headMove(0.0, -1.40);
                if (cset > 5)
                {
                    cekWaktu(16);
                    if (timer)
                    {
                        motion("0");
                    }
                    else if (second > 15 || second < 1)
                    {
                        motion("9");
                        jalanDirection(0.0, 0.0, 0.0);
                    }
                    else
                    {
                        jalanDirection(0.06, 0.0, 0);
                    }
                }
                else
                {
                    setWaktu();
                    cset++;
                }
            }
        }
        else if ((Remaining != 600 && SecondaryState == 0) || (Remaining != 300 && SecondaryState == 2))
        {
            motion("0");
            headMove(0.0, -1.5);
        }
    }

    // int totalDetectLandmarks = 0,
    //     detectLandmarks[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0},
    //     gL = 0, gR = 0, lcL = 0, lcR = 0, pen = 0, xL = 0,
    //     xR = 0, tL = 0, tR = 0, ballDis = 0;
    // double panAngle = 0;

    // // tidak menggunakan kamera zed
    // void sendLokalisasi(int goal, int Ld, int Rd, int Gc, int imu, int Bd, double sudutPan, double Robot_Posx, double Robot_Posy, double odomImu)
    // {
    //     if (Goal_X >= 260 && Goal_X <= 380 && Goal_Y >= 210 && Goal_Y <= 270)
    //     {
    //         if (Ld >= 0 && Ld <= 900 && Rd >= 0 && Rd <= 900)
    //         {
    //             if (abs(Ld - Rd) <= 260 && Gc == 0)
    //             {
    //                 sprintf(dataLokalisasi, "%d,%d,%d,%d,%d,%.f,%.f,%.f,%.f", goal, Ld, Rd, imu, Bd, sudutPan, Robot_Posx, Robot_Posy, odomImu);
    //                 sendto(socklokalisasi, dataLokalisasi, strlen(dataLokalisasi), 0, (struct sockaddr *)&addrlokalisasi, sizeof(addrlokalisasi));
    //             }
    //             else
    //             {
    //                 sprintf(dataLokalisasi, "%d,%d,%d,%d,%d,%.f,%.f,%.f,%.f", goal, -1, -1, imu, Bd, sudutPan, Robot_Posx, Robot_Posy, odomImu);
    //                 sendto(socklokalisasi, dataLokalisasi, strlen(dataLokalisasi), 0, (struct sockaddr *)&addrlokalisasi, sizeof(addrlokalisasi));
    //             }
    //         }
    //         else
    //         {
    //             sprintf(dataLokalisasi, "%d,%d,%d,%d,%d,%.f,%.f,%.f,%.f", goal, -1, -1, imu, Bd, sudutPan, Robot_Posx, Robot_Posy, odomImu);
    //             sendto(socklokalisasi, dataLokalisasi, strlen(dataLokalisasi), 0, (struct sockaddr *)&addrlokalisasi, sizeof(addrlokalisasi));
    //         }
    //     }
    //     else
    //     {
    //         sprintf(dataLokalisasi, "%d,%d,%d,%d,%d,%.f,%.f,%.f,%.f", goal, -1, -1, imu, Bd, sudutPan, Robot_Posx, Robot_Posy, odomImu);
    //         sendto(socklokalisasi, dataLokalisasi, strlen(dataLokalisasi), 0, (struct sockaddr *)&addrlokalisasi, sizeof(addrlokalisasi));
    //     }
    //     // printf("  dataLokalisasi = %s\n", dataLokalisasi);
    // }

    // void *sendData(void *argument)
    // { // send socket lokalisasi
    //     while (1)
    //     {
    //         // sleep(1);
    //         usleep(500000);
    //         sendLokalisasi(goalSide, Goal_LD, Goal_RD, Goal_C, 360 - sendAngle, Ball_D, 57.29 * headPan, robotPos_X, robotPos_Y, ArchSinTeng);
    //     }
    // }

    // // Use Zed camera
    // void sendLokalisasi(int goal, int gLd, int gRd, int lLd, int lRd, int xLd, int xRd, int tLd, int tRd, int imu, int Bd, double sudutPan, double Robot_Posx, double Robot_Posy) {
    // 	if (gLd > 0 && gLd <= 900 || gRd > 0 && gRd <= 900 || lLd > 0 && lLd <= 900 || lRd > 0 && lRd <= 900) {	//saat nilai gawang kiri || gawang kanan || Lcross kiri || Lcross kanan

    //                 if (gLd > 0 && gLd <= 900) {	//jarak gawang kiri
    //                          detectLandmarks[0] = 1;
    // 	                 gL = gLd;
    //                 } else {
    // 	                detectLandmarks[0] = 0;
    // 	                gL = -1;
    //                 }

    //                 if (gRd > 0 && gRd <= 900) {	//jarak gawang kanan
    // 	                detectLandmarks[1] = 1;
    //        		        gR = gRd;
    //                 } else {
    //         	        detectLandmarks[1] = 0;
    //         	        gR = -1;
    //                 }

    // 		if (inversValue == false) {	//invers true ketika robot balik

    // 		        if (lLd > 0 && lLd <= 900) {	//jarak Lcross kiri
    // 			        detectLandmarks[2] = 1;
    // 			        lcL = lLd;
    // 		        } else {
    // 			        detectLandmarks[2] = 0;
    // 			        lcL = -1;
    // 		        }

    // 		        if (lRd > 0 && lRd <= 900) {	//jarak Lcross kanan
    // 			        detectLandmarks[3] = 1;
    // 			        lcR = lRd;
    // 		        } else {
    // 			        detectLandmarks[3] = 0;
    // 			        lcR = -1;
    // 		        }

    // 		        if (xLd > 0 && xLd <= 500) {	//Jarak Xcross kiri
    // 			        detectLandmarks[5] = 1;
    // 			        xL = xLd;
    // 		        } else {
    // 			        detectLandmarks[5] = 0;
    // 			        xL = -1;
    // 		        }

    // 		        if (xRd > 0 && xRd <= 500) {	//Jarak Xcross kanan
    // 			        detectLandmarks[6] = 1;
    // 			        xR = xRd;
    // 		        } else {
    // 			        detectLandmarks[6] = 0;
    // 			        xR = -1;
    // 		        }

    // 		        if (tLd > 0 && tLd <= 500) {	//Jarak Tcross kiri
    // 			        detectLandmarks[7] = 1;
    // 			        tL = tLd;
    // 		        } else {
    // 			        detectLandmarks[7] = 0;
    // 			        tL = -1;
    // 		        }

    // 		        if (tRd > 0 && tRd <= 500) {	//Jarak Tcross kanan
    // 			        detectLandmarks[8] = 1;
    // 			        tR = tRd;
    // 		        } else {
    // 			        detectLandmarks[8] = 0;
    // 			        tR = -1;
    // 		        }
    // 		} else {
    // 			lcL = lcR = xL = xR = tL = tR = 0;
    // 		        detectLandmarks[2] = detectLandmarks[3] =  detectLandmarks[4] = detectLandmarks[5] = detectLandmarks[6] = detectLandmarks[7] = detectLandmarks[8] = 0;
    // 		}
    //         } else {
    // 		gL = gR = lcL = lcR = xL = xR = tL = tR = 0;
    //                 detectLandmarks[0] = detectLandmarks[1] = detectLandmarks[2] = detectLandmarks[3] =  detectLandmarks[4] =
    //                 detectLandmarks[5] = detectLandmarks[6] = detectLandmarks[7] = detectLandmarks[8] = 0;
    //         }
    // 	if (Bd > 0 && Bd <= 900) {
    // 		ballDis = Bd;
    // 	} else {
    // 		ballDis = -1;
    // 	}

    // 	panAngle = imu + sudutPan;
    // 	totalDetectLandmarks = detectLandmarks[0] + detectLandmarks[1] + detectLandmarks[2] + detectLandmarks[3] + detectLandmarks[4] + detectLandmarks[5] + detectLandmarks[6] + detectLandmarks[7] + detectLandmarks[8];
    // 	sprintf(dataLokalisasi, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.f,%.f,%.f", totalDetectLandmarks, goal, gL, gR, lcL, lcR, xL, xR, tL, tR, ballDis, panAngle, Robot_Posx, Robot_Posy);	//fulldata
    // 	sendto(socklokalisasi, dataLokalisasi, strlen(dataLokalisasi), 0,(struct sockaddr *) &addrlokalisasi, sizeof(addrlokalisasi));
    //        	// printf("  dataLokalisasi = %s\n", dataLokalisasi);

    // }

    // void * sendData(void * argument) { //send socket lokalisasi
    //         while (1) {
    // 		usleep(500000);
    // 		sendLokalisasi(goalSide, Goal_LD, Goal_RD, Lcross_LD, Lcross_RD, Xcross_LD, Xcross_RD, Tcross_LD, Tcross_RD, 360-sendAngle, Ball_D, 57.29*headPan, robotPos_X, robotPos_Y);

    //         }
    // }

    // main
    //  Coordinations ===============================================================================
    int delayTrackGoal = 0;
    bool goalFound = false,
         breaked = false;
    void myTurn()
    {
        motion("9");
        // printf("  MYTURN MYTURN MYTURN MYTURN !!!!\n");
        // if (useFollowSearchGoal && useSearchGoal) {
        if (useFollowSearchGoal && followSearchAktif)
        {
            if (angle >= -80 && angle <= 80)
            { // angle dan jarak terpenuhi
                if (chotto > 20)
                {
                    FollowGoal = true;
                }
                else
                {
                    chotto++;
                }
            }

            if (FollowGoal)
            {
                Activated = false;
                if (useCoordination)
                {
                    semeh = 1;
                    sendRobotCoordinationData(robotNumber, robotStatus, 232, Grid, 1, semeh, GridBall, backIn);
                }

                if (!searchGoalFinish)
                { // pertama
                    if (!ballposTracked)
                    { // pertama
                        if (ballLost(20))
                        {
                            if (waiting > 40)
                            {
                                resetCase1();
                                stateCondition = 1;
                            }
                            else
                            {
                                tiltSearchBall(0.0);
                            }
                            waiting++;
                            Walk(0.0, 0.0, 0.0);
                        }
                        else
                        {
                            trackBall();
                            if (headTilt > -1.4 && !useNearFollowSearchGoal)
                            { // kelewatan -1.4
                                delay = 0;
                                tunggu = 0;

                                prediksiGoalPan = 0;
                                saveAngle = 0;

                                goalFound = false;
                                goalSearch = true;

                                Walk(jalan, 0.0, 0.0); // kejar

                                ballposTracked = true;
                                searchGoalFinish = true;
                            }
                            else if (headTilt >= -1.6 && headPan >= -0.1 && headPan <= 0.1)
                            { // 0.1
                                delay = 0;

                                Walk(jalan, 0.0, 0.0);
                                // followBall(0);

                                if (delayTrackBall > 20)
                                {
                                    ballposTracked = true;
                                }
                                else
                                {
                                    delayTrackBall++;
                                }
                            }
                            else
                            { // masih jauh
                                delayTrackBall = 0;

                                prediksiGoalPan = 0;
                                saveAngle = 0;

                                goalFound = false;
                                goalSearch = false;

                                followBall(0);
                            }
                        }
                    }
                    else
                    { // kedua
                        if (!goalSearch)
                        { // pertama
                            tunggu = 0;
                            if (delay > 5)
                            {
                                cekWaktu(6);
                                if (ballTilt < -1.6)
                                { // jarak jauh
                                    Walk(0.06, 0.0, 0.0);
                                }
                                else if (ballTilt < -1.5 && ballTilt >= -1.6)
                                { // jarak sedang
                                    if (second < 4.5)
                                    { // sambil kejar //4.5
                                        Walk(0.05, 0.0, 0.0);
                                    }
                                    else
                                    { // space sebelum sampai ke bola (biar gk nabrak bola)
                                        Walk(0.0, 0.0, 0.0);
                                    }
                                }
                                else
                                { // jarak dekat
                                    if (second < 3)
                                    { // sambil kejar //3.5 // 3
                                        Walk(0.05, 0.0, 0.0);
                                    }
                                    else
                                    { // space sebelum sampai ke bola (biar gk nabrak bola)
                                        Walk(0.0, 0.0, 0.0);
                                    }
                                }

                                if (goalLost(20))
                                { // pertama
                                    delayTrackGoal = 0;
                                    if (!timer)
                                    { // pertama
                                        if (second <= 2.0)
                                        {                             // search goal sambil kejar
                                            predictGoal(angle, -2.0); // tengah
                                        }
                                        else if (second > 4.0)
                                        {
                                            predictGoal(angle + 30, -2.0); // kanan
                                        }
                                        else
                                        {
                                            predictGoal(angle - 30, -2.0); // kiri
                                        }
                                    }
                                    else
                                    { // kedua
                                        prediksiGoalPan = 0;
                                        saveAngle = 0;

                                        if (ballTilt < -1.6)
                                        {
                                            loadBallLocation(0.2); // 0.15
                                        }
                                        else if (ballTilt >= -1.5 && ballTilt >= -1.6)
                                        {
                                            loadBallLocation(0.4); // 0.35
                                        }
                                        else
                                        {
                                            loadBallLocation(0.6);
                                        }

                                        goalFound = false;
                                        goalSearch = true;
                                    }
                                }
                                else
                                { // kedua
                                    if (timer || delayTrackGoal > 50)
                                    { // kedua
                                        if (((saveAngle - prediksiGoalPan) > 90) || ((saveAngle - prediksiGoalPan) < -90))
                                        {
                                            prediksiGoalPan = 0;
                                            saveAngle = 0;
                                        }

                                        if (ballTilt < -1.6)
                                        {
                                            loadBallLocation(0.2); // 0.15
                                        }
                                        else if (ballTilt >= -1.5 && ballTilt >= -1.6)
                                        {
                                            loadBallLocation(0.4); // 0.35
                                        }
                                        else
                                        {
                                            loadBallLocation(0.6);
                                        }

                                        goalFound = true;
                                        goalSearch = true;
                                    }
                                    else
                                    { // pertama
                                        trackGoal();
                                        prediksiArahGoal();
                                        saveSudutImu();
                                        if (useUpdateCoordinate)
                                        {
                                            updateCoordinatFromVision();
                                        }
                                        delayTrackGoal++;
                                    }
                                }
                            }
                            else
                            {
                                trackBall();
                                Walk(kejar, 0.0, 0.0);
                                setWaktu();
                                delayTrackGoal = 0;
                                delay++;
                            }
                        }
                        else
                        { // kedua
                            if (tunggu < 5)
                            { // 20 //pertama
                                trackBall();
                                delay = 0;
                                waiting = 0;
                                robotDirection = false;
                                setWaktu();
                                tunggu++;
                            }
                            else
                            { // kedua
                                if (ballLost(20))
                                {
                                    if (waiting > 40)
                                    {
                                        resetCase1();
                                        stateCondition = 1;
                                    }
                                    else
                                    {
                                        tiltSearchBall(0.0);
                                    }
                                    waiting++;
                                    Walk(0.0, 0.0, 0.0);
                                }
                                else
                                {
                                    trackBall();
                                    waiting = 0;

                                    if (goalFound == true)
                                    {
                                        if (robotDirection && headPan >= -0.2 && headPan <= 0.2)
                                        { // kedua //0.2
                                            // if (headTilt >= (cSekarang - 0.2)) {
                                            Walk(0.0, 0.0, 0.0);
                                            searchGoalFinish = true;
                                            //} else {
                                            //	if (headTilt >= -1.0) {
                                            //		ballPositioning(0.0, cSekarang, 0.12);
                                            //	} else {
                                            //		followBall(0);
                                            //	}
                                            //}
                                        }
                                        else
                                        { // pertama
                                            if (headTilt >= cAktif && headPan >= -0.2 && headPan <= 0.2)
                                            { // kedua //0.2
                                                if (delay > 5)
                                                {
                                                    cekWaktu(20);
                                                    if (timer)
                                                    {
                                                        robotDirection = true;
                                                    }
                                                    else
                                                    {
                                                        if (((saveAngle - prediksiGoalPan) > -30) && ((saveAngle - prediksiGoalPan) < 30))
                                                        {
                                                            if (prediksiGoalPan > 0)
                                                            {
                                                                errorGoal = -abs((saveAngle - prediksiGoalPan) * 0.4);
                                                            } // 0.3 //0.33 //0.4
                                                            else
                                                            {
                                                                errorGoal = abs((saveAngle - prediksiGoalPan) * 0.4);
                                                            } // 0.3 //0.33 //0.4
                                                            if (errorGoal >= 20)
                                                            {
                                                                errorGoal = 20;
                                                            }
                                                        }
                                                        else if (((saveAngle - prediksiGoalPan) >= 30) && ((saveAngle - prediksiGoalPan) < 60) || ((saveAngle - prediksiGoalPan) > -60) && ((saveAngle - prediksiGoalPan) <= -30))
                                                        {
                                                            if (prediksiGoalPan > 0)
                                                            {
                                                                errorGoal = -abs((saveAngle - prediksiGoalPan) * 0.3);
                                                            } // 0.3 //0.33 //0.4
                                                            else
                                                            {
                                                                errorGoal = abs((saveAngle - prediksiGoalPan) * 0.3);
                                                            } // 0.3 //0.33 //0.4
                                                            if (errorGoal >= 20)
                                                            {
                                                                errorGoal = 20;
                                                            }
                                                        }
                                                        else
                                                        {
                                                            errorGoal = 0;
                                                            // if (prediksiGoalPan > 0) { errorGoal = -abs((saveAngle-prediksiGoalPan) * 0.33); } //- //0.3 //0.33 //0.4
                                                            // else { errorGoal = abs((saveAngle-prediksiGoalPan) * 0.33); } //- //0.3 //0.33 //0.4
                                                            // if (errorGoal >= 20) { errorGoal = 20; }
                                                        }

                                                        if (useSideKick)
                                                        {
                                                            if (prediksiGoalPan >= 45)
                                                            {                 // kiri
                                                                modeKick = 4; // tendangSamping
                                                                Imu(90 + (saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
                                                            }
                                                            else if (prediksiGoalPan <= -45)
                                                            {                 // kanan
                                                                modeKick = 3; // tendangSamping
                                                                Imu(-90 + (saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
                                                            }
                                                            else
                                                            {
                                                                modeKick = tendangJauh;
                                                                Imu((saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
                                                                lastDirection = angle;
                                                            }
                                                        }
                                                        else
                                                        {
                                                            modeKick = tendangJauh;
                                                            Imu((saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
                                                            lastDirection = angle;
                                                        }
                                                    }
                                                }
                                                else
                                                {
                                                    setWaktu();
                                                    robotDirection = false;
                                                    delay++;
                                                }
                                            }
                                            else
                                            { // pertama
                                                delay = 0;
                                                followBall(0);
                                            }
                                        }
                                    }
                                    else
                                    {
                                        searchGoalFinish = true;
                                    }
                                }
                            }
                        }
                    }
                }
                else
                { // kedua (next case)
                    if (ballLost(20))
                    {
                        if (tunggu > 40)
                        {
                            resetCase1();
                            stateCondition = 1;
                        }
                        else
                        {
                            tiltSearchBall(0.0);
                        }
                        tunggu++;
                        Walk(0.0, 0.0, 0.0);
                    }
                    else
                    {
                        trackBall();
                        tunggu = 0;

                        if (headTilt >= cAktif)
                        {
                            if (goalFound)
                            {
                                resetCase5();
                                stateCondition = 5;
                            }
                            else
                            {
                                resetCase7();
                                stateCondition = 7;
                            }
                        }
                        else
                        {
                            followBall(0);
                        }
                    }
                }
            }
            else
            { // jika gawang dibelakang (next case)
                if (ballLost(20))
                {
                    if (useCoordination)
                    {
                        semeh = 232;
                        sendRobotCoordinationData(robotNumber, robotStatus, stateCondition, Grid, 0, semeh, GridBall, backIn);
                    }
                    if (tunggu > 40)
                    {
                        resetCase1();
                        stateCondition = 1;
                    }
                    else
                    {
                        tiltSearchBall(0.0);
                    }
                    tunggu++;
                    Walk(0.0, 0.0, 0.0);
                }
                else
                {
                    trackBall();
                    if (useCoordination)
                    {
                        semeh = 1;
                        sendRobotCoordinationData(robotNumber, robotStatus, 232, Grid, 1, semeh, GridBall, backIn);
                    }
                    tunggu = 0;

                    if (headTilt >= cAktif)
                    {
                        resetCase7();
                        stateCondition = 7;
                    }
                    else
                    {
                        if (useLocalization && useUpdateCoordinate)
                        {
                            updateCoordinatFromVision();
                        }
                        followBall(0);
                    }
                }
            }
        }
        else
        { // without follow search goal (next case)
            if (ballLost(20))
            {
                if (useCoordination)
                {
                    semeh = 232;
                    sendRobotCoordinationData(robotNumber, robotStatus, stateCondition, Grid, 0, semeh, GridBall, backIn);
                }
                if (tunggu > 40)
                {
                    resetCase1();
                    stateCondition = 1;
                }
                else
                {
                    tiltSearchBall(0.0);
                }
                tunggu++;
                Walk(0.0, 0.0, 0.0);
            }
            else
            {
                trackBall();
                if (useCoordination)
                {
                    semeh = 1;
                    sendRobotCoordinationData(robotNumber, robotStatus, 232, Grid, 1, semeh, GridBall, backIn);
                }
                tunggu = 0;

                if (headTilt >= cAktif)
                {
                    resetCase7();
                    stateCondition = 7;
                }
                else
                {
                    if (useLocalization && useUpdateCoordinate)
                    {
                        updateCoordinatFromVision();
                    }
                    followBall(0);
                }
            }
        }
    }

    int delayWalkWaiting = 0,
        delayWaitRotate = 0,
        robot1pos_X = 0,
        robot1pos_Y = 0,
        robot3pos_X = 0,
        robot3pos_Y = 0,
        robot1Column = 0,
        robot3Column = 0,
        gridTarget = 0;

    void waitingTurn()
    {
        if (useLocalization)
        {
            printf("  WAITINGTURN WAITINGTURN WAITINGTURN WAITINGTURN !!!!\n");
            // cek kembali status si eksekutor
            if (robotNumber == 1)
            {
                if (robot3State == 1)
                {
                    robot3exeCutor = false;
                }
            }
            else if (robotNumber == 3)
            {
                if (robot1State == 1)
                {
                    robot1exeCutor = false;
                }
            }

            // pergerakkan kepala & state
            if (ballLost(20))
            {
                if (useCoordination)
                {
                    semeh = 232;
                    sendRobotCoordinationData(robotNumber, robotStatus, stateCondition, Grid, 0, semeh, GridBall, backIn);
                }

                if (robotNumber == 1)
                {
                    if (robot3State == 1)
                    {
                        if (tunggu > 40)
                        {
                            resetCase1();
                            stateCondition = 1;
                        }
                        else
                        {
                            searchKe = 0;
                            tiltSearchBall(0.0);
                        }
                        tunggu++;
                    }
                    else
                    {
                        threeSearchBall();
                    }
                }
                else if (robotNumber == 3)
                {
                    if (robot1State == 1)
                    {
                        if (tunggu > 40)
                        {
                            resetCase1();
                            stateCondition = 1;
                        }
                        else
                        {
                            searchKe = 0;
                            tiltSearchBall(0.0);
                        }
                        tunggu++;
                    }
                    else
                    {
                        threeSearchBall();
                    }
                }
            }
            else
            {
                trackBall();
                if (useCoordination)
                {
                    sendRobotCoordinationData(robotNumber, robotStatus, stateCondition, Grid, 1, semeh, GridBall, backIn);
                }
                delay = 0;
                tunggu = 0;
            }

            // kalkulasi posisi robot & pergerakkan robot
            if (robotNumber == 1)
            {
                if (robot3exeCutor)
                {
                    // Baca koordinat robot eksekusi berdasarkan Grid data
                    robot3pos_X = convertGridX(robot3GridPosition, 0);
                    robot3pos_Y = convertGridY(robot3GridPosition, 0);

                    // Ubah grid pos jadi column
                    if (robot3GridPosition % 6 == 0)
                    {
                        robot3Column = robot3GridPosition / 6;
                    }
                    else
                    {
                        robot3Column = (robot3GridPosition / 6) + 1;
                    }

                    if (robot3Column >= 9)
                    {
                        if (robot3pos_Y < 0)
                        {
                            gridTarget = ((robot3Column - 1) * 6) - 2;
                        }
                        else if (robot3pos_Y >= 0)
                        {
                            gridTarget = ((robot3Column - 1) * 6) - 3;
                        }
                    }
                    else
                    {
                        if (robot3pos_Y < 0)
                        {
                            gridTarget = ((robot3Column)*6) + 4;
                        }
                        else if (robot3pos_Y >= 0)
                        {
                            gridTarget = ((robot3Column)*6) + 3;
                        }
                    }

                    if (delayWalkWaiting > 10)
                    {
                        if (doneMoved)
                        {
                            if (delayWaitRotate > 5)
                            {
                                if (posRotateNew)
                                {
                                    motion("0");
                                }
                                else
                                {
                                    motion("9");
                                    rotateBodyImuNew(ArchSinEnemy);
                                }
                            }
                            else
                            {
                                motion("9");
                                Walk(0.0, 0.0, 0.0);
                                posRotateNew = false;
                                delayWaitRotate++;
                            }
                            // Cek kembali posisi robot eksekusi, jika diluar range refresh kemudian jalan lagi
                            // if (robot1exeCutor) {
                            //	if (Grid > ((robot1Column-1)*6) && (Grid < ((robot1Column+1)*6))) {
                            //		motion("0");
                            //	} else {
                            //		refreshMoveGrid();
                            //	}
                            // } else if (robot3exeCutor) {
                            //	if (Grid > ((robot1Column-1)*6) && (Grid < ((robot1Column+1)*6))) {
                            //		motion("0");
                            //	} else {
                            //		refreshMoveGrid();
                            //	}
                            // }
                        }
                        else
                        {
                            motion("9");
                            moveGrid(gridTarget, 0, 0);
                        }
                    }
                    else
                    {
                        motion("9");
                        Walk(0.0, 0.0, 0.0);
                        refreshMoveGrid();
                        delayWalkWaiting++;
                    }
                }
                else
                {
                    motion("0");
                    delayWalkWaiting = 0,
                    delayWaitRotate = 0,
                    robot3pos_X = 0,
                    robot3pos_Y = 0,
                    robot3Column = 0,
                    gridTarget = 0;
                }
            }
            else if (robotNumber == 3)
            {
                if (robot1exeCutor)
                {
                    // Baca koordinat robot eksekusi berdasarkan Grid data
                    robot1pos_X = convertGridX(robot1GridPosition, 0);
                    robot1pos_Y = convertGridY(robot1GridPosition, 0);

                    // Ubah grid pos jadi column
                    if (robot1GridPosition % 6 == 0)
                    {
                        robot1Column = robot1GridPosition / 6;
                    }
                    else
                    {
                        robot1Column = (robot1GridPosition / 6) + 1;
                    }

                    if (robot1Column >= 9)
                    {
                        if (robot1pos_Y < 0)
                        {
                            gridTarget = ((robot1Column - 1) * 6) - 2;
                        }
                        else if (robot3pos_Y >= 0)
                        {
                            gridTarget = ((robot1Column - 1) * 6) - 3;
                        }
                    }
                    else
                    {
                        if (robot1pos_Y < 0)
                        {
                            gridTarget = ((robot1Column)*6) + 4;
                        }
                        else if (robot1pos_Y >= 0)
                        {
                            gridTarget = ((robot1Column)*6) + 3;
                        }
                    }

                    if (delayWalkWaiting > 10)
                    {
                        if (doneMoved)
                        {
                            if (delayWaitRotate > 5)
                            {
                                if (posRotateNew)
                                {
                                    motion("0");
                                }
                                else
                                {
                                    motion("9");
                                    rotateBodyImuNew(ArchSinEnemy);
                                }
                            }
                            else
                            {
                                motion("9");
                                Walk(0.0, 0.0, 0.0);
                                posRotateNew = false;
                                delayWaitRotate++;
                            }

                            // Cek kembali posisi robot eksekusi, jika diluar range refresh kemudian jalan lagi
                            // if (robot1exeCutor) {
                            //	if (Grid > ((robot1Column-1)*6) && (Grid < ((robot1Column+1)*6))) {
                            //		motion("0");
                            //	} else {
                            //		refreshMoveGrid();
                            //	}
                            // } else if (robot3exeCutor) {
                            //	if (Grid > ((robot1Column-1)*6) && (Grid < ((robot1Column+1)*6))) {
                            //		motion("0");
                            //	} else {
                            //		refreshMoveGrid();
                            //	}
                            // }
                        }
                        else
                        {
                            motion("9");
                            moveGrid(gridTarget, 0, 0);
                        }
                    }
                    else
                    {
                        motion("9");
                        Walk(0.0, 0.0, 0.0);
                        refreshMoveGrid();
                        delayWalkWaiting++;
                    }
                }
                else
                {
                    motion("0");
                    delayWalkWaiting = 0,
                    delayWaitRotate = 0,
                    robot1pos_X = 0,
                    robot1pos_Y = 0,
                    robot1Column = 0,
                    gridTarget = 0;
                }
            }
        }
        else
        {
            if (ballLost(20))
            {
                if (useCoordination)
                {
                    semeh = 232;
                    sendRobotCoordinationData(robotNumber, robotStatus, stateCondition, Grid, 0, semeh, GridBall, backIn);
                }
                if (tunggu > 40)
                {
                    resetCase1();
                    stateCondition = 1;
                }
                else
                {
                    searchKe = 0;
                    tiltSearchBall(0.0);
                    Walk(0.0, 0.0, 0.0);
                }
                tunggu++;
            }
            else
            {
                trackBall();
                if (useCoordination)
                {
                    sendRobotCoordinationData(robotNumber, robotStatus, stateCondition, Grid, 1, semeh, GridBall, backIn);
                }

                setWaktu();
                delay = 0;
                tunggu = 0;
                robotDirection = false;

                // if (headTilt > -1.3 && headPan >= -0.4 && headPan <= 0.4) {
                //	Walk(-0.03, 0.0, 0.0);
                // } else if (headTilt >= -1.5 && headTilt <= -1.3 && headPan >= -0.4 && headPan <= 0.4) {
                if (headTilt > -1.5 && headPan >= -0.4 && headPan <= 0.4)
                {
                    // if (angle >= 0) { Imu(90, -1.4); }
                    // else { Imu(-90, -1.4); }
                    if (angle >= 0)
                    { // Imu 90
                        if (angle > 80 && angle < 100)
                        {
                            if (headTilt > -1.3)
                            {
                                bodyXImu = -0.03;
                                alfaImu = 0.0;
                            }
                            else
                            {
                                bodyXImu = alfaImu = 0.0;
                            }
                            bodyYImu = errorCPosPan * 0.06;
                        }
                        else if ((angle <= 90) && (angle >= -90))
                        {
                            rotateDirec(-1, -1.4); // printf("  rotate ke kanan\n\n");
                        }
                        else
                        {
                            rotateDirec(1, -1.4); // printf("  rotate ke kiri\n\n");
                        }
                        Walk(bodyXImu, bodyYImu, alfaImu);
                    }
                    else
                    { // Imu -90
                        if (angle < -80 && angle > -100)
                        {
                            if (headTilt > -1.3)
                            {
                                bodyXImu = -0.03;
                                alfaImu = 0.0;
                            }
                            else
                            {
                                bodyXImu = alfaImu = 0.0;
                            }
                            bodyYImu = errorCPosPan * 0.06;
                        }
                        else if ((angle <= 90) && (angle >= -90))
                        {
                            rotateDirec(1, -1.4); // printf("  rotate ke kiri\n\n");
                        }
                        else
                        {
                            rotateDirec(-1, -1.4); // printf("  rotate ke kanan\n\n");
                        }
                        Walk(bodyXImu, bodyYImu, alfaImu);
                    }
                }
                else
                {
                    if (useLocalization && useUpdateCoordinate)
                    {
                        updateCoordinatFromVision();
                    }
                    followBall(0);
                }
            }
        }
    }

    // main
    void yourTurn()
    {
        if (robotNumber == 1)
        {
            if ( // jika robot saya lebih dekat dengan bola dari robot lain
                (semeh < robot2DBall) &&
                (semeh < robot3DBall) &&
                (semeh < robot4DBall) &&
                (semeh < robot5DBall))
            {
                myTurn();
            }
            else
            { // jalan Imu, jika terlalu dekat maka mundur, jika terlalu jauh maka kejar
                waitingTurn();
            }
        }
        else if (robotNumber == 2)
        {
            if ( // jika robot saya lebih dekat dengan bola dari robot lain
                (semeh < robot1DBall) &&
                (semeh < robot3DBall) &&
                (semeh < robot4DBall) &&
                (semeh < robot5DBall))
            {
                myTurn();
            }
            else
            { // jalan Imu, jika terlalu dekat maka mundur, jika terlalu jauh maka kejar
                waitingTurn();
            }
        }
        else if (robotNumber == 3)
        {
            if ( // jika robot saya lebih dekat dengan bola dari robot lain
                (semeh < robot1DBall) &&
                (semeh < robot2DBall) &&
                (semeh < robot4DBall) &&
                (semeh < robot5DBall))
            {
                myTurn();
            }
            else
            { // jalan Imu, jika terlalu dekat maka mundur, jika terlalu jauh maka kejar
                waitingTurn();
            }
        }
        else if (robotNumber == 4)
        {
            if ( // jika robot saya lebih dekat dengan bola dari robot lain
                (semeh < robot1DBall) &&
                (semeh < robot2DBall) &&
                (semeh < robot3DBall) &&
                (semeh < robot5DBall))
            {
                myTurn();
            }
            else
            { // jalan Imu, jika terlalu dekat maka mundur, jika terlalu jauh maka kejar
                waitingTurn();
            }
        }
        else if (robotNumber == 5)
        {
            if ( // jika robot saya lebih dekat dengan bola dari robot lain
                (semeh < robot1DBall) &&
                (semeh < robot2DBall) &&
                (semeh < robot3DBall) &&
                (semeh < robot4DBall))
            {
                myTurn();
            }
            else
            { // jalan Imu, jika terlalu dekat maka mundur, jika terlalu jauh maka kejar
                waitingTurn();
            }
        }
    }

    int gommen = 0, maxRef = 800,
        countR1 = 0, countR2 = 0, countR3 = 0, countR4 = 0, countR5 = 0,
        lastR1StateBall = 0, lastR2StateBall = 0, lastR3StateBall = 0, lastR4StateBall = 0, lastR5StateBall = 0;
    void refreshComm()
    {
        if (countR1 >= maxRef || countR2 >= maxRef || countR3 >= maxRef || countR4 >= maxRef || countR5 >= maxRef)
        {
            // printf("\n.....................................................................refresh\n");
            refresh = true;
            if (robotNumber != 1)
            {
                robot1Id = robot1FBall = robot1State = robot1Status = 0;
                robot1DBall = 232;
            }
            if (robotNumber != 2)
            {
                robot2Id = robot2FBall = robot2State = robot2Status = 0;
                robot2DBall = 232;
            }
            if (robotNumber != 3)
            {
                robot3Id = robot3FBall = robot3State = robot3Status = 0;
                robot3DBall = 232;
            }
            if (robotNumber != 4)
            {
                robot4Id = robot4FBall = robot4State = robot4Status = 0;
                robot4DBall = 232;
            }
            if (robotNumber != 5)
            {
                robot5Id = robot5FBall = robot5State = robot5Status = 0;
                robot5DBall = 232;
            }
            countR1 = countR2 = countR3 = countR4 = countR5 = 0;
        }
        else
        {
            refresh = false;
            if (robotNumber != 1)
            {
                if (robot1State != 1 && robot1State != 150 && robot1State != 50 && robot1State != 90 && robot1State != 100 && lastR1StateBall == robot1State)
                {
                    countR1++;
                }
                else
                {
                    countR1 = 0;
                    lastR1StateBall = robot1State;
                }
            }
            if (robotNumber != 2)
            {
                if (robot2State != 1 && robot2State != 150 && robot2State != 50 && robot2State != 90 && robot2State != 100 && lastR2StateBall == robot2State)
                {
                    countR2++;
                }
                else
                {
                    countR2 = 0;
                    lastR2StateBall = robot2State;
                }
            }
            if (robotNumber != 3)
            {
                if (robot3State != 1 && robot3State != 150 && robot3State != 50 && robot3State != 90 && robot3State != 100 && lastR3StateBall == robot3State)
                {
                    countR3++;
                }
                else
                {
                    countR3 = 0;
                    lastR3StateBall = robot3State;
                }
            }
            if (robotNumber != 4)
            {
                if (robot4State != 1 && robot4State != 150 && robot4State != 50 && robot4State != 90 && robot4State != 100 && lastR4StateBall == robot4State)
                {
                    countR4++;
                }
                else
                {
                    countR4 = 0;
                    lastR4StateBall = robot4State;
                }
            }
            if (robotNumber != 5)
            {
                if (robot5State != 1 && robot5State != 150 && robot5State != 50 && robot5State != 90 && robot5State != 100 && lastR5StateBall == robot5State)
                {
                    countR5++;
                }
                else
                {
                    countR5 = 0;
                    lastR5StateBall = robot5State;
                }
            }
        } // printf("\n  %d,%d,%d,%d,%d\n", countR1, countR2, countR3, countR4, countR5);
    }

    void backToCoordinations()
    {
        if (robotNumber == 1)
        {
            if ( // jika ada robot lain yang sudah masuk case eksekusi
                (robot2State == 7 || robot2State == 3 || robot2State == 8 || robot2State == 4 || robot2State == 5 || robot2State == -10 || robot2State == 10 || robot2State == 20 || robot2State == 30) ||
                (robot3State == 7 || robot3State == 3 || robot3State == 8 || robot3State == 4 || robot3State == 5 || robot3State == -10 || robot3State == 10 || robot3State == 20 || robot3State == 30) ||
                (robot4State == 7 || robot4State == 3 || robot4State == 8 || robot4State == 4 || robot4State == 5 || robot4State == -10 || robot4State == 10 || robot4State == 20 || robot4State == 30) ||
                (robot5State == 7 || robot5State == 3 || robot5State == 8 || robot5State == 4 || robot5State == 5 || robot5State == -10 || robot5State == 10 || robot5State == 20 || robot5State == 30))
            {
                resetCase2();
                stateCondition = 2;
            }
        }
        else if (robotNumber == 2)
        {
            if ( // jika ada robot lain yang sudah masuk case eksekusi
                (robot1State == 7 || robot1State == 3 || robot1State == 8 || robot1State == 4 || robot1State == 5 || robot1State == -10 || robot1State == 10 || robot1State == 20 || robot1State == 30) ||
                (robot3State == 7 || robot3State == 3 || robot3State == 8 || robot3State == 4 || robot3State == 5 || robot3State == -10 || robot3State == 10 || robot3State == 20 || robot3State == 30) ||
                (robot4State == 7 || robot4State == 3 || robot4State == 8 || robot4State == 4 || robot4State == 5 || robot4State == -10 || robot4State == 10 || robot4State == 20 || robot4State == 30) ||
                (robot5State == 7 || robot5State == 3 || robot5State == 8 || robot5State == 4 || robot5State == 5 || robot5State == -10 || robot5State == 10 || robot5State == 20 || robot5State == 30))
            {
                resetCase2();
                stateCondition = 2;
            }
        }
        else if (robotNumber == 3)
        {
            if ( // jika ada robot lain yang sudah masuk case eksekusi
                (robot1State == 7 || robot1State == 3 || robot1State == 8 || robot1State == 4 || robot1State == 5 || robot1State == -10 || robot1State == 10 || robot1State == 20 || robot1State == 30) ||
                (robot2State == 7 || robot2State == 3 || robot2State == 8 || robot2State == 4 || robot2State == 5 || robot2State == -10 || robot2State == 10 || robot2State == 20 || robot2State == 30) ||
                (robot4State == 7 || robot4State == 3 || robot4State == 8 || robot4State == 4 || robot4State == 5 || robot4State == -10 || robot4State == 10 || robot4State == 20 || robot4State == 30) ||
                (robot5State == 7 || robot5State == 3 || robot5State == 8 || robot5State == 4 || robot5State == 5 || robot5State == -10 || robot5State == 10 || robot5State == 20 || robot5State == 30))
            {
                resetCase2();
                stateCondition = 2;
            }
        }
        else if (robotNumber == 4)
        {
            if ( // jika ada robot lain yang sudah masuk case eksekusi
                (robot1State == 7 || robot1State == 3 || robot1State == 8 || robot1State == 4 || robot1State == 5 || robot1State == -10 || robot1State == 10 || robot1State == 20 || robot1State == 30) ||
                (robot2State == 7 || robot2State == 3 || robot2State == 8 || robot2State == 4 || robot2State == 5 || robot2State == -10 || robot2State == 10 || robot2State == 20 || robot2State == 30) ||
                (robot3State == 7 || robot3State == 3 || robot3State == 8 || robot3State == 4 || robot3State == 5 || robot3State == -10 || robot3State == 10 || robot3State == 20 || robot3State == 30) ||
                (robot5State == 7 || robot5State == 3 || robot5State == 8 || robot5State == 4 || robot5State == 5 || robot5State == -10 || robot5State == 10 || robot5State == 20 || robot5State == 30))
            {
                resetCase2();
                stateCondition = 2;
            }
        }
        else if (robotNumber == 5)
        {
            if ( // jika ada robot lain yang sudah masuk case eksekusi
                (robot1State == 7 || robot1State == 3 || robot1State == 8 || robot1State == 4 || robot1State == 5 || robot1State == -10 || robot1State == 10 || robot1State == 20 || robot1State == 30) ||
                (robot2State == 7 || robot2State == 3 || robot2State == 8 || robot2State == 4 || robot2State == 5 || robot2State == -10 || robot2State == 10 || robot2State == 20 || robot2State == 30) ||
                (robot3State == 7 || robot3State == 3 || robot3State == 8 || robot3State == 4 || robot3State == 5 || robot3State == -10 || robot3State == 10 || robot3State == 20 || robot3State == 30) ||
                (robot4State == 7 || robot4State == 3 || robot4State == 8 || robot4State == 4 || robot4State == 5 || robot4State == -10 || robot4State == 10 || robot4State == 20 || robot4State == 30))
            {
                resetCase2();
                stateCondition = 2;
            }
        }
    }

    // normal search Ball ========================================================================
    bool posRotasi = false;
    int rotasi = 0;
    void rotateSearchBall(int rotate)
    {
        if (rotate >= 0)
        {
            rotasi = rotate - 180;
            sudut();

            setPoint1 = 20 + rotasi;  // 20
            setPoint2 = -20 + rotasi; // 20

            if (angle > setPoint2 && angle < setPoint1)
            {
                posRotasi = true;
            }
            else
            {
                // jalanDirection(0.0, 0.0, rotasi);
                Walk(0.0, 0.0, 0.27);
            }
        }
        else
        {
            rotasi = rotate + 180;
            sudut();

            setPoint1 = 20 + rotasi;  // 20
            setPoint2 = -20 + rotasi; // 20

            if (angle > setPoint2 && angle < setPoint1)
            {
                posRotasi = true;
            }
            else
            {
                // jalanDirection(0.0, 0.0, rotasi);
                Walk(0.0, 0.0, 0.27);
            }
        }
    }

    bool firstRotate = false,
         secondRotate = false,
         thirdRotate = false,
         fourthRotate = false,

         firstWalk = false,
         secondWalk = false;
    void normalSearchBall()
    {
        // mode1
        /* if (!firstRotate) { //ini step yang pertama kali dilakukan saat masuk case 0
            if (searchKe >= 5) {
                Walk(0.0, 0.0, 0.0); //X, Y, W
                firstRotate  = true;
                searchKe = 0;
            } else if (searchKe >= 2 && searchKe < 5) { printf("  2.........\n\n");
                SearchBall(2);
                //tiltSearchBall(0.0);
                Walk(0.0, 0.0, 0.15);
            } else { printf("  1.........\n\n");
                SearchBall(2);
                Walk(0.0, 0.0, 0.0); //X, Y, W
            }
        } else { //setelah rotate pertama tidak dapat, maka cari sambil jalan(dengan imu)
            if (!secondRotate) { //yang pertama kali dilakukan
                if (searchKe >= 3) { printf("  4.........\n\n");
                    SearchBall(2);
                    //tiltSearchBall(0.0);
                    Walk(0.0, 0.0, 0.12);

                    if (searchKe >= 5) {
                        searchKe = 0;
                        secondRotate  = true;
                    }
                } else { printf("  3.........\n\n");
                    SearchBall(2);
                    Walk(kejar, 0.0, 0.0);
                }
            } else { //jalan dengan arah sebaliknya
                if (searchKe >= 6) { printf("  6.........\n\n");
                    SearchBall(2);
                    if (searchKe >= 8) {
                        if (searchKe > 12) {
                            jalanDirection(kejar, 0.0, 0);
                        } else {
                            //searchKe = 0;
                            //firstRotate = secondRotate = false;
                            Walk(kejar, 0.0, 0.0);
                        }
                    } else {
                        //tiltSearchBall(0.0);
                        Walk(0.0, 0.0, 0.12);
                    }
                } else { printf("  5.........\n\n");
                    SearchBall(2);
                    Walk(kejar, 0.0, 0.0);
                }
            }
        } */

        // mode 2
        if (!firstRotate)
        {
            if (matte > 5)
            {
                if (searchKe == 1)
                {
                    if (posRotasi)
                    {
                        motion("0");
                        // Walk(0.0, 0.0, 0.0);
                        // jalanDirection(0.0, 0.0, rotasi);     //before
                        matte = 0;
                        firstRotate = true;
                    }
                    else
                    {
                        motion("9");
                        // headMove(0.0, -1.6);
                        threeSearchBall();
                        rotateSearchBall(saveAngle);
                    }
                }
                else
                {
                    motion("0");
                    saveSudutImu();
                    threeSearchBall();
                    // Walk(0.0, 0.0, 0.0);  //before
                }
            }
            else
            {
                posRotasi = false;
                sabar = 0;
                searchKe = 0;
                matte++;
            }
        }
        else
        {
            if (!secondRotate)
            {
                if (matte > 5)
                {
                    if (searchKe == 1)
                    {
                        if (posRotasi)
                        {
                            motion("0");
                            // Walk(0.0, 0.0, 0.0);
                            // jalanDirection(0.0, 0.0, rotasi);     //before
                            matte = 0;
                            secondRotate = true;
                        }
                        else
                        {
                            motion("9");
                            // headMove(0.0, -1.6);
                            threeSearchBall();
                            rotateSearchBall(saveAngle);
                        }
                    }
                    else
                    {
                        motion("0");
                        saveSudutImu();
                        threeSearchBall();
                        // Walk(0.0, 0.0, 0.0);
                        // jalanDirection(0.0, 0.0, saveAngle);  //before
                    }
                }
                else
                {
                    posRotasi = false;
                    sabar = 0;
                    searchKe = 0;
                    matte++;
                }
            }
            else
            {
                if (!firstWalk)
                {
                    if (matte > 5)
                    {
                        if (searchKe == 2)
                        {
                            motion("0");
                            // posRotasi = false;
                            saveSudutImu();
                            // Walk(0.0, 0.0, 0.0);
                            // jalanDirection(0.0, 0.0, saveAngle);  //before
                            matte = 0;
                            firstWalk = true;
                        }
                        else
                        {
                            motion("9");
                            threeSearchBall();
                            // Walk(kejar, 0.0, 0.0);
                            jalanDirection(kejar, 0.0, rotasi);
                        }
                    }
                    else
                    {
                        posRotasi = false;
                        sabar = 0;
                        searchKe = 0;
                        matte++;
                    }
                }
                else
                {
                    if (!thirdRotate)
                    {
                        if (matte > 5)
                        {
                            if (posRotasi)
                            {
                                motion("0");
                                // Walk(0.0, 0.0, 0.0);
                                // jalanDirection(0.0, 0.0, rotasi);
                                // sabar = 0;
                                // searchKe = 0;
                                matte = 0;
                                thirdRotate = true;
                            }
                            else
                            {
                                motion("9");
                                threeSearchBall();
                                rotateSearchBall(saveAngle);
                            }
                        }
                        else
                        {
                            posRotasi = false;
                            sabar = 0;
                            searchKe = 0;
                            matte++;
                        }
                    }
                    else
                    {
                        if (!secondWalk)
                        {
                            if (matte > 5)
                            {
                                if (searchKe == 4)
                                {
                                    motion("0");
                                    // posRotasi = false;
                                    saveSudutImu();
                                    // Walk(0.0, 0.0, 0.0);
                                    // jalanDirection(0.0, 0.0, rotasi);
                                    matte = 0;
                                    secondWalk = true;
                                }
                                else
                                {
                                    motion("9");
                                    threeSearchBall();
                                    // Walk(kejar, 0.0, 0.0);
                                    jalanDirection(kejar, 0.0, rotasi);
                                }
                            }
                            else
                            {
                                posRotasi = false;
                                sabar = 0;
                                searchKe = 0;
                                matte++;
                            }
                        }
                        else
                        {
                            if (!fourthRotate)
                            {
                                if (matte > 5)
                                {
                                    if (posRotasi)
                                    {
                                        motion("0");
                                        // Walk(0.0, 0.0, 0.0);
                                        // jalanDirection(0.0, 0.0, rotasi);
                                        // sabar = 0;
                                        // searchKe = 0;
                                        matte = 0;
                                        fourthRotate = true;
                                    }
                                    else
                                    {
                                        motion("9");
                                        threeSearchBall();
                                        rotateSearchBall(saveAngle);
                                    }
                                }
                                else
                                {
                                    posRotasi = false;
                                    sabar = 0;
                                    searchKe = 0;
                                    matte++;
                                }
                            }
                            else
                            {
                                if (matte > 5)
                                {
                                    motion("9");
                                    threeSearchBall();
                                    // Walk(kejar, 0.0, 0.0);
                                    jalanDirection(kejar, 0.0, rotasi);
                                }
                                else
                                {
                                    posRotasi = false;
                                    sabar = 0;
                                    searchKe = 0;
                                    matte++;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    bool rotatePertama = false,
         rotateKedua = false,
         rotateKetiga = false,
         finishSearch = false,
         searchOne = false,
         jalanPertama = false,
         jalanKedua = false;
    int waitSearchBall = 0;
    void normalSearchBallGrid(int valueGrid1, int valueOffSetX1, int valueOffSetY1, int valueGrid2, int valueOffSetX2, int valueOffSetY2)
    {
        if (!rotatePertama)
        {
            if (useLocalization && useUpdateCoordinate)
            {
                updateCoordinatFromVision();
            }
            if (waitSearchBall > 5)
            {
                if (searchKe >= 1)
                {
                    if (posRotasi)
                    {
                        motion("0");
                        waitSearchBall = 0;
                        rotatePertama = true;
                    }
                    else
                    {
                        motion("9");
                        threeSearchBall();
                        rotateSearchBall(saveAngle);
                    }
                }
                else
                {
                    // motion("9");
                    // Walk(0, 0, 0);
                    motion("0");
                    threeSearchBall();
                    saveSudutImu();
                }
            }
            else
            {
                sabar = 0;
                posRotasi = false;
                searchKe = 0;
                waitSearchBall++;
            }
        }
        else
        {
            if (!rotateKedua)
            {
                if (useLocalization && useUpdateCoordinate)
                {
                    updateCoordinatFromVision();
                }
                if (waitSearchBall > 5)
                {
                    if (searchKe >= 1)
                    {
                        if (posRotasi)
                        {
                            motion("0");
                            waitSearchBall = 0;
                            rotateKedua = true;
                        }
                        else
                        {
                            motion("9");
                            threeSearchBall();
                            rotateSearchBall(saveAngle);
                        }
                    }
                    else
                    {
                        motion("0");
                        threeSearchBall();
                        saveSudutImu();
                    }
                }
                else
                {
                    sabar = 0;
                    posRotasi = false;
                    searchKe = 0;
                    waitSearchBall++;
                }
            }
            else
            {
                if (!jalanPertama)
                {
                    if (waitSearchBall > 5)
                    {
                        if (searchKe >= 1)
                        {
                            if (doneMoved)
                            {
                                motion("0");
                                waitSearchBall = 0;
                                jalanPertama = true;
                            }
                            else
                            {
                                motion("9");
                                threeSearchBall();
                                moveGrid(valueGrid1, valueOffSetX1, valueOffSetY1);
                            }
                        }
                        else
                        {
                            motion("0");
                            threeSearchBall();
                        }
                    }
                    else
                    {
                        refreshMoveGrid();
                        sabar = 0;
                        searchKe = 0;
                        waitSearchBall++;
                    }
                }
                else
                {
                    if (!rotateKetiga)
                    {
                        if (useLocalization && useUpdateCoordinate)
                        {
                            updateCoordinatFromVision();
                        }
                        if (waitSearchBall > 5)
                        {
                            if (searchKe >= 1)
                            {
                                if (posRotasi)
                                {
                                    motion("0");
                                    waitSearchBall = 0;
                                    rotateKetiga = true;
                                }
                                else
                                {
                                    motion("9");
                                    threeSearchBall();
                                    rotateSearchBall(saveAngle);
                                }
                            }
                            else
                            {
                                motion("0");
                                saveSudutImu();
                                threeSearchBall();
                            }
                        }
                        else
                        {
                            sabar = 0;
                            searchKe = 0;
                            posRotasi = false;
                            waitSearchBall++;
                        }
                    }
                    else
                    {
                        if (!jalanKedua)
                        {
                            if (waitSearchBall > 5)
                            {
                                if (searchKe >= 1)
                                {
                                    if (doneMoved)
                                    {
                                        motion("0");
                                        waitSearchBall = 0;
                                        jalanKedua = true;
                                    }
                                    else
                                    {
                                        motion("9");
                                        threeSearchBall();
                                        moveGrid(valueGrid2, valueOffSetX2, valueOffSetY2);
                                    }
                                }
                                else
                                {
                                    motion("0");
                                    threeSearchBall();
                                }
                            }
                            else
                            {
                                refreshMoveGrid();
                                sabar = 0;
                                searchKe = 0;
                                waitSearchBall++;
                            }
                        }
                        else
                        {
                            if (!finishSearch)
                            { // rotate
                                if (waitSearchBall > 5)
                                {
                                    if (searchKe >= 1)
                                    {
                                        if (posRotasi)
                                        {
                                            motion("0");
                                            waitSearchBall = 0;
                                            finishSearch = true;
                                        }
                                        else
                                        {
                                            motion("9");
                                            threeSearchBall();
                                            rotateSearchBall(saveAngle);
                                        }
                                    }
                                    else
                                    {
                                        motion("0");
                                        saveSudutImu();
                                        threeSearchBall();
                                    }
                                }
                                else
                                {
                                    sabar = 0;
                                    searchKe = 0;
                                    posRotasi = false;
                                    waitSearchBall++;
                                }
                            }
                            else
                            {
                                if (useLocalization && useUpdateCoordinate)
                                {
                                    updateCoordinatFromVision();
                                }
                                if (!searchOne)
                                {
                                    // printf("SEARCH ONE FALSEEEEEE\n");
                                    if (waitSearchBall > 5)
                                    {
                                        if (searchKe >= 2)
                                        {
                                            if (posRotasi)
                                            {
                                                motion("0");
                                                waitSearchBall = 0;
                                                searchOne = true;
                                            }
                                            else
                                            {
                                                motion("9");
                                                threeSearchBall();
                                                rotateSearchBall(saveAngle);
                                            }
                                        }
                                        else
                                        {
                                            motion("0");
                                            threeSearchBall();
                                            saveSudutImu();
                                        }
                                    }
                                    else
                                    {
                                        sabar = 0;
                                        searchKe = 0;
                                        posRotasi = false;
                                        waitSearchBall++;
                                    }
                                }
                                else if (searchOne)
                                {
                                    printf("SEARCH ONE TRUEEEEE\n");
                                    if (waitSearchBall > 5)
                                    {
                                        if (searchKe >= 2)
                                        {
                                            if (posRotasi)
                                            {
                                                motion("0");
                                                waitSearchBall = 0;
                                                searchOne = false;
                                            }
                                            else
                                            {
                                                motion("9");
                                                threeSearchBall();
                                                rotateSearchBall(saveAngle);
                                            }
                                        }
                                        else
                                        {
                                            motion("0");
                                            threeSearchBall();
                                            saveSudutImu();
                                        }
                                    }
                                    else
                                    {
                                        sabar = 0;
                                        searchKe = 0;
                                        posRotasi = false;
                                        waitSearchBall++;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    // Reset Variabel ============================================================================
    void resetCaseAwal()
    { // strategi awal
        kondisiBola =
            elapsedTime =
                second =
                    reset =
                        cntOke1 =
                            cntOke2 =
                                cntUlang =
                                    delay =
                                        count =
                                            tunggu =
                                                Waktu =
                                                    stateCondition =
                                                        Timing = 0;

        modeKick = tendangJauh;

        ulang =
            followSearchAktif = true;

        timer =
            oke =
                tracked =
                    posRotate =
                        robotDirection =
                            tendang = false;
    }

    void resetCase1()
    { // search ball
        delayWaitBall =
            reset =
                tracked = false;
        i = 1;
        if (useLocalization)
        {
            if (stateCondition != 1 || finishSearch)
            {
                // tipe data int
                saveAngle =
                    tiltPos =
                        searchKe =
                            waitSearchBall =
                                rotasi =
                                    sabar = 0;
                // tiltPos		= 0;

                // tipe data boolean
                rotatePertama =
                    rotateKedua =
                        rotateKetiga =
                            jalanPertama =
                                jalanKedua =
                                    searchOne =
                                        finishSearch = false;
            }
        }
        else
        {
            saveAngle =
                searchKe = // counting berapa kali search
                rotasi =
                    sabar =
                        matte =
                            tiltPos = 0;

            posRotasi =
                firstRotate =
                    secondRotate =
                        thirdRotate =
                            fourthRotate =
                                firstWalk =
                                    secondWalk = false;
        }
    }

    void resetCase2()
    { // cek koordinasi, follow, n cari gawang
        searchKe =
            tunggu =
                delay =
                    delayTrackBall =
                        delayWalkWaiting =
                            // elapsedTime		=
                            // second			=
            reset =
                waiting =
                    errorGoal =
                        goalPan =
                            saveAngle =
                                prediksiGoalPan =
                                    chotto = 0;

        modeKick = tendangJauh;

        Activated = true;

        exeCutor =
            FollowGoal =
                searchGoalFinish =
                    // timer			=
            ballPos =
                goalSearch =
                    ballposTracked =
                        goalFound =
                            breaked =
                                robotDirection = false;
        refreshMoveGrid();
    }

    void resetCase7()
    { // imus
        elapsedTime =
            second =
                reset =
                    delay =
                        delayTrackGoal =
                            tunggu = 0;

        modeKick = tendangJauh;

        timer =
            robotDirection = false;
    }

    void resetCase3()
    { // dribble
        searchKe =
            tunggu =
                countDribble = 0;

        robotDirection = false;
    }

    void resetCase8()
    { // robot must to check direction again after dribble
        searchKe =
            tunggu =
                delay = 0;

        robotDirection = false;
    }

    void resetCase4()
    { // search goal
        searchKe =
            tunggu =
                Rotate =
                    delay =
                        waiting =
                            elapsedTime =
                                second =
                                    reset =
                                        count =
                                            bodyP_ControllerG =
                                                bodyTrueG =
                                                    delayTrueG =
                                                        errorGoal =
                                                            goalPan =
                                                                saveAngle =
                                                                    prediksiGoalPan = 0;

        modeKick = tendangJauh;

        searchGoalFinish =
            timer =
                ballPos =
                    goalSearch =
                        rotateGoal =
                            ballposTracked =
                                errorGoal =
                                    robotDirection = false;
    }

    void resetCase5()
    { // kick
        searchKe =
            delay =
                tunggu = 0;

        kanan =
            kiri =
                ballPos =
                    tendang = false;
    }

    void resetCase6()
    { // search after kick
        searchKe =
            tunda =
                tunggu =
                    elapsedTime =
                        second =
                            delay =
                                confirmsBall =
                                    sumTilt =
                                        waitTracking =
                                            countTilt = 0;

        neckX = true;

        searchRectangle =
            timer = false;
    }

    void resetOdometry()
    {
        robotPos_X =
            robotPos_Y =
                ArchSinEnemy =
                    csmKiri =
                        csmKanan =
                            walkTot =
                                deltaPos_X =
                                    varCount =
                                        deltaPos_Y =
                                            walkTotMun = 0;
    }

    void refreshMoveLokalisasi()
    {
        cL =     // count lock for atribut target
            cD = // count peralihan dari rotate ke jalan
            cR = // count peralihan dari break ke rotate
            cZ = cV = cN = cY =
                cB = 0; // count untuk break ketika sudah didalam setpoint target
        cX = false,
        donePosition = false;
    }

    void refreshMoveGrid()
    {
        posRotateNew = false;
        doneMoved = false;
        countMoveGrid1 =
            countMoveGrid2 =
                countMoveGrid3 = 0;
    }

    void resetKoordinasiRobotBalik()
    {
        balikTengah = false;
        balikSampingKanan = false;
        balikSampingKiri = false;
    }

    void refreshRecelerLokalisasi()
    {
        RobotCoor_X =
            RobotCoor_Y =
                BallCoor_X =
                    BallCoor_Y = 0;
    }

    void resetAllLokalisasiVariable()
    {
        lockLeftBack =
            lockRightBack =
                kembaliMasuk =
                    lockMidBack =
                        awalMasuk = false;

        // resetOdometry();
        // refreshRecelerLokalisasi();
        refreshMoveLokalisasi();
    }

    void resetAllVariable()
    { // setelah tendang
        resetCaseAwal();
        resetCase1();
        resetCase2();
        resetCase3();
        resetCase4();
        resetCase5();
        resetCase6();
        resetCase7();
        resetCase8();

        kembaliMasuk = false,
        awalMasuk = false,
        lockLeftBack = false,
        lockMidBack = false,
        lockRightBack = false;
        resetKoordinasiRobotBalik();
        resetAllLokalisasiVariable();
    }

private:
    rclcpp::Subscription<bfc_msgs::msg::Button>::SharedPtr button_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr bounding_boxes_;
    rclcpp::Subscription<bfc_msgs::msg::Coordination>::SharedPtr robot1Subscription_;
    rclcpp::Subscription<bfc_msgs::msg::Coordination>::SharedPtr robot2Subscription_;
    rclcpp::Subscription<bfc_msgs::msg::Coordination>::SharedPtr robot3Subscription_;
    rclcpp::Subscription<bfc_msgs::msg::Coordination>::SharedPtr robot4Subscription_;
    rclcpp::Subscription<bfc_msgs::msg::Coordination>::SharedPtr robot5Subscription_;
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr gameControllerSubscription_;
    rclcpp::Publisher<bfc_msgs::msg::Coordination>::SharedPtr robotCoordination_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_mot_;
    rclcpp::Publisher<bfc_msgs::msg::HeadMovement>::SharedPtr cmd_head_;
    rclcpp::TimerBase::SharedPtr timer_;

    int msg_strategy, msg_kill, msg_roll, msg_pitch, msg_yaw;

    int deltaPos_X, deltaPos_Y, RobotAngle, ArchSinEnemy, ArchSinTeng, walkTot, ArchSinTeam, csmKiri, csmKanan, walkTotMun,
        RobotPos_X, RobotPos_Y;

    int robot1Id, robot1Status, robot1State, robot1GridPosition, robot1FBall, robot1DBall, robot1GridBall, robot1BackIn;
    int robot2Id, robot2Status, robot2State, robot2GridPosition, robot2FBall, robot2DBall, robot2GridBall, robot2BackIn;
    int robot3Id, robot3Status, robot3State, robot3GridPosition, robot3FBall, robot3DBall, robot3GridBall, robot3BackIn;
    int robot4Id, robot4Status, robot4State, robot4GridPosition, robot4FBall, robot4DBall, robot4GridBall, robot4BackIn;
    int robot5Id, robot5Status, robot5State, robot5GridPosition, robot5FBall, robot5DBall, robot5GridBall, robot5BackIn;

    int Ball_X, Ball_Y, Ball_W, Ball_H, Ball_D;
    int Goal_X, Goal_Y, Goal_W, Goal_H, Goal_LH, Goal_RH, Goal_C, Goal_LD, Goal_RD;
    int Goal_LX, Goal_LY, Goal_RX, Goal_RY, Xcross_RX, Xcross_RY, Xcross_LX, Xcross_LY;
    int Pinalty_D, Lcross_LD, Lcross_RD, Xcross_LD, Xcross_RD, Tcross_LD, Tcross_RD;
    int GoalLeft_X, GoalLeft_Y, GoalRight_X, GoalRight_Y;
    int RobotCoor_X, RobotCoor_Y, BallCoor_X, BallCoor_Y, HeadingCoor;

    double RobotWalk_X, RobotWalk_Y, RobotWalk_A;

    bool robotGerak, refresh;

    int State, Player, Team,
        FirstHalf,
        Version,
        PacketNumber,
        PlayerTeam,
        GameTipe,
        KickOff,
        SecondaryState,
        DropTeam,
        DropTime,
        Remaining,
        SecondaryTime,
        // ket : 1 = untuk data GameController yang kiri
        //	 2 = untuk data GameController yang kanan
        timNumber1,
        timNumber2,
        timColour1,
        timColour2,
        Score1,
        Score2,
        Penaltyshoot1,
        Penaltyshoot2,
        Singleshoot1,
        Singleshoot2,
        Coachsequence1,
        Coachsequence2,

        Penalty1,
        Penalty2,
        TimeUnpenalis1,
        TimeUnpenalis2,
        YellowCard1,
        YellowCard2,
        RedCard1,
        RedCard2;

    int team, barelang_color, dropball;
    /////////////////////////////////////////////////////////
    ///////////////////Variable Global///////////////////////
    /////////////////////////////////////////////////////////
    int robotNumber,
        stateCondition = 0,
        firstStateCondition = 0, // switch strategy

        stateGameController = 0,
        lastStateGameController = 0,
        stateChange = 0,

        kickOff = 0,       // parsing tambahan ketika gameController bermasalah
        lastKickOff = 0,   // parsing tambahan ketika gameController bermasalah
        kickOffChange = 0, // parsing tambahan ketika gameController bermasalah

        secRemaining = 0,
        lastSecRemaining = 0,
        secRemainingChange = 0,

        state,     // kill n run
        lastState, // kill n run
        wait = 0,

        delay = 0,         // search goal case 4
        delayWaitBall = 0, // search ball case 0
        countBearing = 0,  // Imu erorr
        countDribble = 0,  // lama dribble
        tunda = 0,
        tunggu = 0,
        waiting = 0,
        waitTracking = 0,
        reset = 0,
        delayTrackBall = 0,
        matte = 0,
        chotto = 0,

        modeKick = 1,

        saveAngle = 0,
        lastDirection = 0,

        countHilang = 0,

        confirmsBall = 0,
        countTilt = 0,
        sumTilt = 0,

        Strategi = 0,

        countInitPos = 0, // lock initial pos untuk pickup
        robotStatus = 1,  // Robot Aktif

        varCount = 0, // variable counting untuk case 1001
        countDef = 0, // variable counting untuk case 130 & 140
        cntCal0 = 0,
        cntCal1 = 0,
        cntCal2 = 0,
        cntCal3 = 0,
        cntCal4 = 0,
        cntCal5 = 0,
        cntCal6 = 0,
        odometryCaliState = 0,
        strategyNumber = 0,
        killNrun = 0,
        delayLife = 0,
        cntAwal = 0,
        cntJalan0 = 0,
        cntJalan1 = 0,
        cntJalan2 = 0,
        cntWaitBall = 0,
        cntStabilize0 = 0,
        cntStabilize1 = 0,
        cntDitempat0 = 0,
        cntDitempat1 = 0,
        cntDitempat2 = 0,
        cntDitempat3 = 0,
        modeImu = 0,
        modeTendang = 0,
        stateImu = 0,
        lastStateCondition = 0,

        // New Variable 2020
        modePlay = 0,                                                              // set initial posisi 0 = nasional, 1 = internasional
        varCount1 = 0, varCount2 = 0, varCount3 = 0, varCount4 = 0, varCount5 = 0, // variable counting void moveGrid
        BackIn = 0,                                                                // koordinasi robot balik
        // initialPos_X = 0, initialPos_Y = 0,
        // robotPos_X = 0,	robotPos_Y = 0,
        Grid = 0, // grid posisi robot
        initGrid = 0, offsetX = 0, offsetY = 0,
        ballPos_X = 0, ballPos_Y = 0, // koordinat posisi bola dilapangan
        GridBall = 0;                 // grid posisi bola

    double headPan, // f.Kepala
        headTilt,   // f.Kepala
        posPan,
        posTilt,
        errorPan,  // f.Kepala
        errorTilt, // f.Kepala
        PPan,      // f.Kepala
        PTilt,     // f.Kepala

        ball_panKP,  // PID trackBall
        ball_panKD,  // PID trackBall
        ball_tiltKP, // PID trackBall
        ball_tiltKD, // PID trackBall

        goal_panKP,  // PID trackGoal
        goal_panKD,  // PID trackGoal
        goal_tiltKP, // PID trackGoal
        goal_tiltKD,

        land_panKP,  // PID trackLand
        land_panKD,  // PID trackLand
        land_tiltKP, // PID trackLand
        land_tiltKD, // PID trackLand

        RollCM,
        PitchCM,
        YawCM; // PID trackGoal

    double ballPositioningSpeed,
        pTiltTendang,
        pPanTendang,
        pTiltOper,
        pPanOper,
        cSekarang,
        cAktif,
        posTiltLocal,
        posTiltGoal,
        erorrXwalk,
        erorrYwalk,
        erorrAwalk,
        jalan,
        lari,
        kejar,
        kejarMid,
        kejarMax,
        tinggiRobot,
        outputSudutY1,
        inputSudutY1,
        outputSudutY2,
        inputSudutY2,
        outputSudutX1,
        inputSudutX1,
        outputSudutX2,
        inputSudutX2,
        frame_X,
        frame_Y,
        rotateGoal_x,
        rotateGoal_y,
        rotateGoal_a,

        sendAngle,
        angle,

        robotPos_X,   // om
        initialPos_X, // ov
        robotPos_Y,   // om
        initialPos_Y, // om

        kurama = 0,
        offsetPan = 0;

    int sudutTengah,
        sudutKanan,
        sudutKiri,
        tendangJauh,
        tendangDekat,
        tendangSamping;

    bool play = false,
         firstTimes = true, // first strategy player
        zeroState = false,  // send ball is not found (for coordinations)
        tracked = false,    // search ball
        pickUp = false,     // strategi setelah pick up
        searchRectangle = false,
         FollowGoal = false,

         masukKiri = false,
         masukKanan = false,
         backPosition = false,
         switched = false,
         manual = false,
         signIn = false,

         kanan = false,
         kiri = false,

         followSearchAktif = false,
         Activated = false,

         jatuh = false,
         exeCutor = false,
         lowBatt = false,

         robot1exeCutor = false, // koordinasi
        robot2exeCutor = false,  // koordinasi
        robot3exeCutor = false,  // koordinasi
        robot4exeCutor = false,  // koordinasi
        robot5exeCutor = false,  // koordinasi

        useVision, // f.setting
        useRos,
         useGameController,      // f.setting
        useCoordination,         // f.setting
        useLocalization,         // f.setting
        useFollowSearchGoal,     // f.setting
        useImu,                  // f.setting
        useDribble,              // f.setting
        dribbleOnly,             // f.setting
        useSearchGoal,           // f.setting
        useSideKick,             // f.setting
        useLastDirection,        // f.setting
        useNearFollowSearchGoal, // f.setting
        useUpdateCoordinate,     // f.setting
        usePenaltyStrategy;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<main_strategy>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}