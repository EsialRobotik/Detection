#include "Display.h"
#include <cstdio>
#include <QtGui/QApplication>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QPointF>
#include <QVector>
#include <cstdlib>
#include <time.h> 
#include <assert.h> 


namespace robotik
{
  Display::Display(unsigned int win_x_max_px, unsigned int win_y_max_px,
      unsigned int disp_max_x_mm, unsigned int disp_max_y_mm,
      unsigned int table_size_x_mm, unsigned int table_size_y_mm,
      Lidar *lidar)  : QThread(NULL), scene_(0), lidar_(lidar)
  {
    assert(lidar->is_running());
    windows_size_px_ = coord_t(win_x_max_px, win_y_max_px);

    max_displayable_dist_mm_ = coord_t(disp_max_x_mm, disp_max_y_mm);

    table_size_mm_ = coord_t(table_size_x_mm, table_size_y_mm);

    center_px_ = coord_t(windows_size_px_.x/2, windows_size_px_.y/2);

    table_orig_px = coord_t( center_px_.x - mm2pxOverX(table_size_mm_.x/2), center_px_.y - mm2pxOverY(table_size_mm_.y/2));
    table_size_px  = coord_t( mm2pxOverX(table_size_mm_.x), mm2pxOverY(table_size_mm_.y));

    printf("win(%d, %d) px\n", windows_size_px_.x, windows_size_px_.y);
    printf("maxdispl(%d, %d) mm\n", max_displayable_dist_mm_.x, max_displayable_dist_mm_.y);
    printf("tableSize(%d, %d) mm\n", table_size_mm_.x, table_size_mm_.y);
    printf("center(%d, %d) px\n", center_px_.x, center_px_.y);
    printf("tableOrig(%d, %d) px\n", table_orig_px.x, table_orig_px.y);
    printf("tableSize(%d, %d) px\n", table_size_px.x, table_size_px.y); 
  }
  Display::~Display()
  {

  }

  int Display::start_qt_display()
  {
    int argc = 1;
    char *argv[] = {"display", NULL};
    QApplication app(argc, argv );
    QGraphicsView * view = new QGraphicsView();
    scene_ = new QGraphicsScene();
    scene_->setSceneRect(0,0,windows_size_px_.x, windows_size_px_.y);
    view->setScene(scene_);

    connect(this, SIGNAL(updateDisplaySig()), this, SLOT(updateDisplay()));


    view->show();
    start();
    return app.exec();
  }

  void Display::updateData(std::vector<lidar_xy_sample_t> &points, coord_t &robot_pos)
  {
    points_px_.clear();
    robot_position_mm_ = robot_pos;
    robot_position_px_ = coord_t( table_orig_px.x + mm2pxOverX(robot_position_mm_.x), table_orig_px.y + mm2pxOverY(robot_position_mm_.y));


    for(unsigned int i=0; i<points.size(); i++)
    {
      coord_t coord_px( robot_position_px_.x + mm2pxOverX(points[i].x),
          robot_position_px_.y + mm2pxOverY(points[i].y));
      points_px_.push_back(coord_px);
    }
  }

  void Display::updateDisplay()
  {
    scene_->clear();

    // Display table
    scene_->addRect(table_orig_px.x, table_orig_px.y,
        table_size_px.x, table_size_px.y,
        QPen(), QBrush(Qt::lightGray)  );

    // Display Robot
    scene_->addRect(robot_position_px_.x, robot_position_px_.y, 10, 10,
        QPen(), QBrush(Qt::magenta) );

    for(unsigned int i=0; i<points_px_.size(); i++)
    {
      QBrush brush(Qt::green);
      if((points_px_[i].x < table_orig_px.x) || (points_px_[i].x > (table_orig_px.x+table_size_px.x))
          || (points_px_[i].y < table_orig_px.y) || (points_px_[i].y > (table_orig_px.y+table_size_px.y) ))
        brush = QBrush(Qt::red);

      scene_->addEllipse(points_px_[i].x, points_px_[i].y, 3, 3,  QPen(), brush);
    }
  }

  void Display::run()
  {
    while(1)
    {
      std::vector<robotik::lidar_xy_sample_t> samples;
      lidar_->get360degreesXYsamples(samples);
      coord_t robot_pos(100,300);
      updateData(samples, robot_pos);
      emit updateDisplaySig();
    }

  }


  // Convert a distance in mm in pixels in the display over X
  int Display::mm2pxOverX(int x_mm)
  {
      return int
          ( float(x_mm)/float(max_displayable_dist_mm_.x) * (float)windows_size_px_.x);
  }
  // Convert a distance in mm in pixels in the display over Y
  int Display::mm2pxOverY(int y_mm)
  {
    return int
        ( float(y_mm)/float(max_displayable_dist_mm_.y) * (float)windows_size_px_.y);
  }

}
