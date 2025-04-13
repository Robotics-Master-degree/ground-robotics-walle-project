#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseArray.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/Bool.h"
#include "nav_msgs/GetPlan.h"


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


ros::Publisher pub_cmdExploration;
ros::Publisher pub_moveBaseCancelGoal;
ros::Publisher pub_personaAgafada;

bool personaagafada = false;
std_msgs::Bool cmd_exploration;
std_msgs::Bool cmd_personaAgafada;

geometry_msgs::PointStamped sortida_points [1];

move_base_msgs::MoveBaseGoal goal;
ros::ServiceClient check_path ;
nav_msgs::GetPlan srv;

geometry_msgs::PointStamped min_point;
geometry_msgs::PointStamped pose_walle;
move_base_msgs::MoveBaseActionFeedback feedback_;
geometry_msgs::PoseArray persones_a_salvar_;
geometry_msgs::PoseArray persones_salvades_;
actionlib_msgs::GoalID goalid;

int num_persona_agafada;
int num_persones_salvades;
double dist_pers_salvada;

double calculate_euclidean_distance(double x1, double y1, double x2, double y2){
    double x = x1 - x2;
    double y = y1 - y2;
    double dist;
    dist = pow(x,2)+ pow(y,2);
    dist = sqrt(dist);
    return dist;
}

bool find_rescued_person(geometry_msgs::Pose persona, geometry_msgs::PoseArray pers_salvades, double dist){
  double x1,y1,x2,y2;
  int num_pers_salvades = pers_salvades.poses.size();
  x1=persona.position.x;
  y1=persona.position.y;
  bool trobat = false;
  for (int j=0; j<num_pers_salvades; j++){
    x2=pers_salvades.poses[j].position.x;
    y2=pers_salvades.poses[j].position.y;
    double dist_pers_ = calculate_euclidean_distance(x1,y1,x2,y2);
    std::cout << "print distance" << dist_pers_ << std::endl;
    if (abs(dist_pers_) < dist){
        trobat=true;

    }

  }
  //  std::cout << "aquesta persona ja ha estat tractada?"<< trobat;
  return trobat;
}


     int cerca_persona_proxima_2 (geometry_msgs::PoseArray pers_a_salvar, move_base_msgs::MoveBaseActionFeedback fb,geometry_msgs::PoseArray pers_salvades, double dist ){
            int num_persones_a_salvar = pers_a_salvar.poses.size();
            std::cout << "Numero persones a salvar : " << num_persones_a_salvar <<std::endl;
            int num_persones_salvades = pers_salvades.poses.size();
            std::cout << "Numero persones salvades: " << num_persones_salvades <<std::endl;;
            double dist_pers_a_salvar,dist_pers_salvades;
            bool trobat = false;
            int i_= 999;
            double dist_ = 999999;
            //mirem la distància de la persona a salvar amb la posició actual
            for (int i = 0; i < num_persones_a_salvar; i++) {

              //Servei goal a cada punt personaAgafada
              srv.request.start = fb.feedback.base_position;
              srv.request.goal.pose.position.x = pers_a_salvar.poses[i].position.x -0.3;
              srv.request.goal.pose.position.y = pers_a_salvar.poses[i].position.y -0.3;
              srv.request.goal.header.frame_id = "map";
              srv.request.goal.header.stamp = ros::Time::now();
              srv.request.tolerance = 1.0;
              check_path.call(srv);
              dist_pers_a_salvar = srv.response.plan.poses.size();

              std::cout << "Persona n"<< i << "   distancia check_path_a_salvar:  " << dist_pers_a_salvar <<std::endl;;

                if (num_persones_salvades == 0){
                //  std::cout << "primera persona que agafem";
                  if (dist_pers_a_salvar < dist_){
                    i_ = i;
                    dist_ = dist_pers_a_salvar;
              //      std::cout << "i: " << i_ << "dist_" << dist_;
                  }
                  else{
              //      std::cout << "persona " << i << " está més lluny que la mínima";
                  }
                }
                else{
                    trobat = find_rescued_person(pers_a_salvar.poses[i],pers_salvades,dist);
                    if (!trobat){
                      if (abs(dist_pers_a_salvar) < dist_){
                        i_=i;
                        dist_=dist_pers_a_salvar;
                      }
                    }
                }
              }
          //    std::cout << "dist_ :"  << dist_ << std::endl;
          //    std::cout << "i_ :"  << i_ << std::endl;
              return i_;
        }

void cerebro(MoveBaseClient& ac){

  if (persones_salvades_.poses.size()==5){
    std::cout << "------   S'HAN SALVAT 5 PERSONES!!  ---"<<std::endl;
  }

  if (personaagafada == false) { //No hi ha persona Agafada
    //Com que no tenim persona agafada la distancia a perill < 1

    //CODI AQUI

      std::cout << "Case 1: persona no agafada ." << std::endl;
      //Si no tenim persona agafada i hem vist persones anar a buscar
      if (((persones_a_salvar_.poses.size()!= 0) and (persones_salvades_.poses.size() < 5)) and (persones_a_salvar_.poses.size()!=persones_salvades_.poses.size())){
        //parem explorer
        try{
    //      ROS_INFO("cmd_exploration");
          cmd_exploration.data = false;
          std::cout << cmd_exploration << std::endl;
          pub_cmdExploration.publish(cmd_exploration);
        } catch(int e){
          std::cout << cmd_exploration << std::endl;
      //    std::cout << "Exploration in false" << std::endl;
        }

        //Parem goal
        try{
          goalid = feedback_.status.goal_id;
          std::cout << goalid << std::endl;
        //  ROS_INFO("Try to catch feedback from explorer");
          pub_moveBaseCancelGoal.publish(goalid);
        } catch(int e){
          std::cout << feedback_ << std::endl;
    //      std::cout << "Feedback no received" << std::endl;
        }

        //pub_moveBaseCancelGoal.publish(goalid);

        //Anem a buscar la persona més proxima
        num_persona_agafada = cerca_persona_proxima_2(persones_a_salvar_, feedback_, persones_salvades_,dist_pers_salvada);
        std::cout << "Persona trobada, anem a buscar persona: "<< num_persona_agafada <<std::endl;

        //Enviar goal a buscar persona


        //MoveBaseClient ac("move_base", true);
        while(!ac.waitForServer(ros::Duration(5.0)));
        //  ROS_INFO("Waiting for the move_base action server");
          goal.target_pose.header.frame_id = "map";
          goal.target_pose.header.stamp = ros::Time::now();
          //goal.target_pose.pose.position.x =  persones_a_salvar_.poses[num_persona_agafada].position.x - 0.4;
          //goal.target_pose.pose.position.y =  persones_a_salvar_.poses[num_persona_agafada].position.y - 0.4 ;
          //goal.target_pose.pose.position.x =  persones_a_salvar_.poses[num_persona_agafada].position.x + 0.5;
          goal.target_pose.pose.position.x =  persones_a_salvar_.poses[num_persona_agafada].position.x + 0.3;
          goal.target_pose.pose.position.y =  persones_a_salvar_.poses[num_persona_agafada].position.y - 0.5 ;
          goal.target_pose.pose.orientation.w = 1.0;

          //Comprovació path cap al goal

          //Servei goal a cada punt personaAgafada
          srv.request.start = feedback_.feedback.base_position;
          srv.request.goal.pose = goal.target_pose.pose;
          srv.request.goal.header.frame_id = "map";
          srv.request.goal.header.stamp = ros::Time::now();
          srv.request.tolerance = 0.1;
          check_path.call(srv);
      //    std::cout << "Size call service " << srv.response.plan.poses.size() << std::endl;

          if (srv.response.plan.poses.size() == 0){
            std::cout << "Canviem cantó del goal"<<std::endl;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            //goal.target_pose.pose.position.x =  persones_a_salvar_.poses[num_persona_agafada].position.x + 0.5;
            //goal.target_pose.pose.position.y =  persones_a_salvar_.poses[num_persona_agafada].position.y - 0.5 ;
            goal.target_pose.pose.position.x =  persones_a_salvar_.poses[num_persona_agafada].position.x - 0.4;
            goal.target_pose.pose.position.y =  persones_a_salvar_.poses[num_persona_agafada].position.y - 0.4 ;
            goal.target_pose.pose.orientation.w = 1.0;
          }

          //ROS_INFO("Anem a buscar persona  .................................... ");
          std::cout << "Anem a buscar persona " << num_persona_agafada <<std::endl;
          ac.sendGoalAndWait(goal);
          //ac.waitForResult();

          if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            //ROS_INFO("Agafo persona  ...........................................");
            std::cout << "Agafo persona" << num_persona_agafada <<std::endl;
            //ROS_INFO(num_persona_agafada);
            personaagafada = true;
            cmd_personaAgafada.data = true;
            pub_personaAgafada.publish(cmd_personaAgafada);
          }

          else{
            if((ac.getState() == actionlib::SimpleClientGoalState::REJECTED ) or (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)){
              ROS_INFO("Goal REJECTED or ABORTED, provo altre banda");
              personaagafada = false;
              cmd_personaAgafada.data = false;
              pub_personaAgafada.publish(cmd_personaAgafada);
          //    try{
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x =  persones_a_salvar_.poses[num_persona_agafada].position.x - 0.4;
            goal.target_pose.pose.position.y =  persones_a_salvar_.poses[num_persona_agafada].position.y - 0.1;
            goal.target_pose.pose.orientation.w = 1.0;


                ac.sendGoalAndWait(goal);
                //ac.waitForResult();

                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                  //ROS_INFO("Agafo persona  ...........................................");
                  std::cout << "Agafo persona" << num_persona_agafada <<std::endl;
                  //ROS_INFO(num_persona_agafada);
                  personaagafada = true;
                  cmd_personaAgafada.data = true;
                  pub_personaAgafada.publish(cmd_personaAgafada);
                }
                else{
                    ROS_INFO("Goal REJECTED or ABORTED definitiu ");
                }

          //      goalid = feedback_.status.goal_id;
          //      std::cout << goalid << std::endl;
              //  ROS_INFO("Try to catch feedback from explorer");
          //      pub_moveBaseCancelGoal.publish(goalid);
          //    } catch(int e){
          //      std::cout << feedback_ << std::endl;
          //      std::cout << "Feedback no received" << std::endl;
          //    }
          //    ac.sendGoal(goal);
          //    ac.waitForResult();
          //  }
          //  else{
          //    ROS_INFO("Hi ha algo que ha anat malament ........  :(  ");
           }

       }
    }
          //std::cout << "persones_a_salvar_" << persones_a_salvar_ << std::endl;
      else{
          std::cout << "Case 2: Encara no s'ha rebut persona, explorem";
          //Publish exploration true
          cmd_exploration.data = true;
          pub_cmdExploration.publish(cmd_exploration);
      }

  }
  else{ //Persona persona Agafada
      //Si tenim persona agafada i coneixem sortida
      if(sortida_points[0].point.x != 0 and sortida_points[0].point.y != 0){
          std::cout << "Case 3: Coneixo Sortida i porto persona agafada"<<std::endl;

          //Parem explorer
          try{
          //  ROS_INFO("cmd_exploration");
            sleep(5);
            cmd_exploration.data = false;
            std::cout << cmd_exploration << std::endl;
            pub_cmdExploration.publish(cmd_exploration);
          } catch(int e){
            std::cout << cmd_exploration << std::endl;
          //  std::cout << "Exploration in false" << std::endl;
          }
  //Cancel·lo goal
          try{
            goalid = feedback_.status.goal_id;
            std::cout << goalid << std::endl;
        //    ROS_INFO("try to catch feecback 2");
            pub_moveBaseCancelGoal.publish(goalid);
          } catch(int e){
        //    std::cout << feedback_ << std::endl;
        //    std::cout << "Feedback no received" << std::endl;
          }


          //Envio goal sortida
          while(!ac.waitForServer(ros::Duration(5.0)));
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x =   sortida_points[0].point.x + 1.0;
            goal.target_pose.pose.position.y =   sortida_points[0].point.y - 0.2;
            goal.target_pose.pose.orientation.w = 1.0;

            //Servei goal comprovem sortida
            srv.request.start = feedback_.feedback.base_position;
            srv.request.goal = goal.target_pose;
            srv.request.tolerance = 0.4;
            check_path.call(srv);
            std::cout << "Size call service " << srv.response.plan.poses.size() << std::endl;

            if (srv.response.plan.poses.size() == 0){
              ROS_INFO("Canviem cantó del goal de sortida");
              goal.target_pose.header.frame_id = "map";
              goal.target_pose.header.stamp = ros::Time::now();
              goal.target_pose.pose.position.x =   sortida_points[0].point.x + 1.0;
              goal.target_pose.pose.position.y =  sortida_points[0].point.y + 0.2;
              goal.target_pose.pose.orientation.w = 1.0;
            }


            std::cout << "Anem a deixar persona. " <<std::endl;
            ac.sendGoal(goal);
            ac.waitForResult();
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
              std::cout << "Deixo persona."<<std::endl;
              personaagafada = false;
              persones_salvades_.poses.push_back(persones_a_salvar_.poses[num_persona_agafada]);
              std::cout << "num_pers_salvades" << persones_salvades_<<std::endl;
              num_persones_salvades ++;
              cmd_personaAgafada.data = false;
              pub_personaAgafada.publish(cmd_personaAgafada);

            }
            else{
              if((ac.getState() == actionlib::SimpleClientGoalState::REJECTED ) or (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)){
                ROS_INFO("Goal REJECTED or ABORTED");
                personaagafada = true;
                cmd_personaAgafada.data = true;
                pub_personaAgafada.publish(cmd_personaAgafada);}
          //      try{
                  goal.target_pose.header.frame_id = "map";
                  goal.target_pose.header.stamp = ros::Time::now();
                  goal.target_pose.pose.position.x =   sortida_points[0].point.x + 1.5;
                  goal.target_pose.pose.position.y =   sortida_points[0].point.y - 0.5;
                  goal.target_pose.pose.orientation.w = 1.0;
          //        goalid = feedback_.status.goal_id;
          //        std::cout << goalid << std::endl;
          //        ROS_INFO("REJECTED/ABORTED - Ultim recurs");
                  pub_moveBaseCancelGoal.publish(goalid);
          //      } catch(int e){
          //        std::cout << feedback_ << std::endl;
          //        std::cout << "Feedback no received" << std::endl;
          //      }
                ac.sendGoal(goal);
                ac.waitForResult();
          //    }

          //    else{
          //      ROS_INFO("Hi ha algo que ha anat malament ........  :(  ");
          //    }
            }
      }
      else{
          //Publish exploration true
          std::cout << "Cas 4: No conec sortida, exploro"<<std::endl;
          cmd_exploration.data = true;
          pub_cmdExploration.publish(cmd_exploration);
      }
  }

}


void persones_subscriber_callback (geometry_msgs::PoseArray persones_sub)
{
    persones_a_salvar_ = persones_sub;

}


void perill_subscriber_callback (geometry_msgs::PoseArray perill_sub)
{
  //  ROS_INFO("There are some danger zones declared");

}


void sortida_subscriber_callback (geometry_msgs::PoseArray sortida_sub)
{
    sortida_points[0].point.x = sortida_sub.poses[0].position.x;
    sortida_points[0].point.y = sortida_sub.poses[0].position.y;
    sortida_points[0].point.z = 0;
    sortida_points[0].header.frame_id = "map";

}

void move_feedback_callback(move_base_msgs::MoveBaseActionFeedback feedback_sub){
    feedback_ = feedback_sub;
}

main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "cerebro_walle");
    ros::NodeHandle nh;

    ros::Subscriber sub_persones = nh.subscribe ("/output/waypoints/people", 100, persones_subscriber_callback);
    ros::Subscriber sub_sortida = nh.subscribe ("/output/waypoints/wayout", 1, sortida_subscriber_callback);
    ros::Subscriber sub_move_fedback = nh.subscribe("/move_base/feedback", 1, move_feedback_callback);

    check_path = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");

    MoveBaseClient ac("move_base", true);
    move_base_msgs::MoveBaseGoal goal;
    while(!ac.waitForServer(ros::Duration(5.0)));
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    num_persones_salvades = 0;
    dist_pers_salvada = 1.0;
    // Publish state en exploration
    pub_cmdExploration = nh.advertise<std_msgs::Bool>("cmd_exploration", 1);
    pub_moveBaseCancelGoal = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
    pub_personaAgafada = nh.advertise<std_msgs::Bool>("personaAgafada", 1);
    cerebro(ac);
    ros::Rate r(5);
    while(ros::ok()){
      ros::spinOnce();
      cerebro(ac);
  //    pub_personaAgafada.publish(cmd_personaAgafada);
      r.sleep();
    }

    return 0;


}
