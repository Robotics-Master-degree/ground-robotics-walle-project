// Copyright 2018 David V. Lu!!
#ifndef exit_NAVIGATION_LAYERS_PROXEMIC_LAYER_H
#define exit_NAVIGATION_LAYERS_PROXEMIC_LAYER_H
#include <ros/ros.h>
#include <exit_navigation_layers/exit_layer.h>
#include <dynamic_reconfigure/server.h>
#include <exit_navigation_layers/ProxemicLayerConfig.h>

double gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew);
double get_radius(double cutoff, double A, double var);

namespace exit_navigation_layers
{
class ProxemicLayer : public exitLayer
{
public:
  ProxemicLayer()
  {
    layered_costmap_ = NULL;
  }

  virtual void onInitialize();
  virtual void updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

protected:
  void configure(ProxemicLayerConfig &config, uint32_t level);
  double cutoff_, amplitude_, covar_, factor_;
  dynamic_reconfigure::Server<ProxemicLayerConfig>* server_;
  dynamic_reconfigure::Server<ProxemicLayerConfig>::CallbackType f_;
};
}  // namespace exit_navigation_layers

#endif  // exit_NAVIGATION_LAYERS_PROXEMIC_LAYER_H
