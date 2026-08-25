#ifndef DK_SEARCH_RVIZ_PANEL_SEARCH_CONTROL_PANEL_H
#define DK_SEARCH_RVIZ_PANEL_SEARCH_CONTROL_PANEL_H

#include <rviz/panel.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QString>

#include <string>
#include <vector>

namespace dk_search_rviz_panel
{

class SearchControlPanel : public rviz::Panel
{
  Q_OBJECT

public:
  explicit SearchControlPanel(QWidget* parent = nullptr);
  ~SearchControlPanel() override = default;

  void load(const rviz::Config& config) override;
  void save(rviz::Config config) const override;

private Q_SLOTS:
  void setLastKnownMode();
  void setRectBoundaryMode();
  void setPolyBoundaryMode();
  void finishPolyBoundary();
  void undoPoint();
  void clearDraft();
  void publishTask();
  void clearTask();

  void refreshMainRoute();
  void refreshBackupRoute();
  void refreshAllRoutes();
  void refreshMainByRf();
  void refreshBackupByRf();
  void sendRouteToUav();
  void resetMainRoute();
  void goalReached();

  void useB1();
  void useB2();
  void useB3();

  void handleClickedPointQt(double x, double y, double z);
  void handleSearchTaskStatusQt(QString text);
  void handleOrderedGoalSequenceQt(QString text);
  void handleBackupGoalSequenceQt(QString text);
  void handleRfGradientStatusQt(QString text);

private:
  struct Point3
  {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

  enum class InputMode
  {
    IDLE,
    CLICK_LAST_KNOWN,
    CLICK_RECT_BOUNDARY,
    CLICK_POLY_BOUNDARY
  };

  enum class BoundaryType
  {
    NONE,
    RECT,
    POLYGON
  };

  void buildUi();
  void setupRos();

  void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
  void searchTaskStatusCallback(const std_msgs::String::ConstPtr& msg);
  void orderedGoalSequenceCallback(const std_msgs::String::ConstPtr& msg);
  void backupGoalSequenceCallback(const std_msgs::String::ConstPtr& msg);
  void rfGradientStatusCallback(const std_msgs::String::ConstPtr& msg);

  void publishBool(ros::Publisher& pub, bool value = true);
  void selectBackupRoute(const std::string& route_id);

  void setMode(InputMode mode, const QString& hint);
  void updateLabels();
  void publishDraftMarkers();
  void clearDraftMarkers();

  std::string buildTaskJson() const;
  std::string jsonEscape(const std::string& s) const;
  QString boundarySummary() const;
  QString pointSummary(const Point3& p) const;
  QString shortJsonSummary(const QString& text) const;

  visualization_msgs::Marker makeDeleteAllMarker() const;
  visualization_msgs::Marker makeSphereMarker(
      int id,
      const std::string& ns,
      const Point3& p,
      double scale,
      float r,
      float g,
      float b,
      float a) const;
  visualization_msgs::Marker makeTextMarker(
      int id,
      const std::string& ns,
      const Point3& p,
      const std::string& text,
      double scale,
      float r,
      float g,
      float b,
      float a) const;
  visualization_msgs::Marker makeLineStripMarker(
      int id,
      const std::string& ns,
      const std::vector<Point3>& points,
      bool close_loop,
      double width,
      float r,
      float g,
      float b,
      float a) const;

private:
  ros::NodeHandle nh_;

  ros::Publisher set_task_pub_;
  ros::Publisher clear_task_pub_;
  ros::Publisher main_refresh_pub_;
  ros::Publisher backup_refresh_pub_;
  ros::Publisher all_refresh_pub_;
  ros::Publisher select_backup_pub_;
  ros::Publisher goal_reached_pub_;
  ros::Publisher reset_main_pub_;
  ros::Publisher draft_marker_pub_;
  ros::Publisher send_route_pub_;

  ros::Subscriber clicked_point_sub_;
  ros::Subscriber search_task_status_sub_;
  ros::Subscriber ordered_goal_sequence_sub_;
  ros::Subscriber backup_goal_sequence_sub_;
  ros::Subscriber rf_gradient_status_sub_;

  std::string frame_id_ = "map";

  InputMode mode_ = InputMode::IDLE;
  BoundaryType boundary_type_ = BoundaryType::NONE;

  bool has_last_known_ = false;
  Point3 last_known_;
  std::vector<Point3> rect_points_;
  std::vector<Point3> poly_points_;

  QLabel* mode_label_ = nullptr;
  QLabel* hint_label_ = nullptr;
  QLabel* last_label_ = nullptr;
  QLabel* boundary_label_ = nullptr;
  QLabel* main_label_ = nullptr;
  QLabel* backup_label_ = nullptr;
  QLabel* rf_label_ = nullptr;
  QLabel* rf_hint_label_ = nullptr;

  QLineEdit* target_id_edit_ = nullptr;
  QDoubleSpinBox* sigma_spin_ = nullptr;
  QCheckBox* auto_refresh_check_ = nullptr;

  QPlainTextEdit* status_text_ = nullptr;

  int target_index_ = 1;
};

}  // namespace dk_search_rviz_panel

#endif  // DK_SEARCH_RVIZ_PANEL_SEARCH_CONTROL_PANEL_H
