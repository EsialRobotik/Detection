/*
 * DisplayPoints.h
 *
 *  Created on: 22 févr. 2017
 *      Author: T0132956
 */

#ifndef NIARF_DISPLAY_H_
#define NIARF_DISPLAY_H_

#include <iostream>
#include <vector>
#include <QThread>
#include <QVector>
#include "Lidar.h"


class QGraphicsScene;


namespace robotik
{
  struct coord_t
  {
    int x;
    int y;
    coord_t(int x_val, int y_val) : x(x_val), y(y_val) {}
    coord_t()  {}
  } ;

  class Display: public QThread 
  {
    Q_OBJECT

    public:
      Display(unsigned int win_x_max_px, unsigned int win_y_max_px,
          unsigned int disp_max_x_mm, unsigned int disp_max_y_mm,
          unsigned int table_size_x_mm, unsigned int table_size_y_mm,
          Lidar *lidar);

      virtual ~Display();

      int start_qt_display();

    signals:
      void updateDisplaySig();

    public slots:
      void updateDisplay();
      

    protected:
      void run();
 

    private:
      Q_DISABLE_COPY(Display)

      void updateData(std::vector<lidar_xy_sample_t> &points, coord_t &robot_pos);

      // Convert a distance in mm in pixels in the display over X
      int mm2pxOverX(int x_mm);
      // Convert a distance in mm in pixels in the display over Y
      int mm2pxOverY(int y_mm);

      // Taille fenetre QT en pixel
      coord_t windows_size_px_;

      // Distance max afficheable depuis le millieu de la fenetre
      //    en mm
      coord_t max_displayable_dist_mm_;

      // Taille de la table
      //    en mm
      coord_t table_size_mm_;

      // Position du robot
      coord_t robot_position_mm_;  // par rapport à la table en mm
      coord_t robot_position_px_; // par rapport à l'affichage en pixel

      // Centre de la fenetre en pixel
      coord_t center_px_;
      // Position de la table en px
      coord_t table_orig_px;
      coord_t table_size_px;

      // liste des points
      std::vector<coord_t> points_px_; // par rapport à l'affichage en pixel

      QGraphicsScene * scene_;
      Lidar *lidar_;
  };
}
#endif /* NIARF_DISPLAY_H_ */
