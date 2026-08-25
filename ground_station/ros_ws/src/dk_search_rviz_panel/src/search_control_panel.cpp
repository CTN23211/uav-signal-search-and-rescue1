#include <dk_search_rviz_panel/search_control_panel.h>

#include <pluginlib/class_list_macros.h>

#include <QGridLayout>
#include <QHBoxLayout>
#include <QMetaObject>
#include <QRegularExpression>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace dk_search_rviz_panel
{

SearchControlPanel::SearchControlPanel(QWidget* parent)
  : rviz::Panel(parent)
{
  qRegisterMetaType<QString>("QString");
  buildUi();
  setupRos();
  updateLabels();
  publishDraftMarkers();
}

void SearchControlPanel::buildUi()
{
  auto* root = new QVBoxLayout;
  root->setContentsMargins(6, 6, 6, 6);
  root->setSpacing(6);

  auto* title = new QLabel("<b>UAV Search Region + RF Gradient Control</b>");
  root->addWidget(title);

  auto* task_group = new QGroupBox("Task region input");
  auto* task_layout = new QGridLayout;

  task_layout->addWidget(new QLabel("Target ID"), 0, 0);
  target_id_edit_ = new QLineEdit("HELP_001");
  task_layout->addWidget(target_id_edit_, 0, 1, 1, 3);

  auto* region_note = new QLabel("Region-only mode: boundary = last-seen/search area");
  task_layout->addWidget(region_note, 1, 0, 1, 2);

  sigma_spin_ = new QDoubleSpinBox;
  sigma_spin_->setRange(0.5, 30.0);
  sigma_spin_->setDecimals(1);
  sigma_spin_->setSingleStep(0.5);
  sigma_spin_->setValue(3.0);
  sigma_spin_->hide();

  auto_refresh_check_ = new QCheckBox("Refresh after publish");
  auto_refresh_check_->setChecked(true);
  task_layout->addWidget(auto_refresh_check_, 1, 2, 1, 2);

  auto* rect_btn = new QPushButton("Rect Region");
  auto* poly_btn = new QPushButton("Poly Region");
  auto* finish_poly_btn = new QPushButton("Finish Poly");
  auto* undo_btn = new QPushButton("Undo Point");
  auto* clear_draft_btn = new QPushButton("Clear Draft");
  auto* publish_task_btn = new QPushButton("Publish Region");
  auto* clear_task_btn = new QPushButton("Clear Region");

  task_layout->addWidget(rect_btn, 2, 0);
  task_layout->addWidget(poly_btn, 2, 1);
  task_layout->addWidget(finish_poly_btn, 2, 2);
  task_layout->addWidget(publish_task_btn, 2, 3);

  task_layout->addWidget(undo_btn, 3, 0);
  task_layout->addWidget(clear_draft_btn, 3, 1);
  task_layout->addWidget(clear_task_btn, 3, 2);

  connect(rect_btn, &QPushButton::clicked, this, &SearchControlPanel::setRectBoundaryMode);
  connect(poly_btn, &QPushButton::clicked, this, &SearchControlPanel::setPolyBoundaryMode);
  connect(finish_poly_btn, &QPushButton::clicked, this, &SearchControlPanel::finishPolyBoundary);
  connect(undo_btn, &QPushButton::clicked, this, &SearchControlPanel::undoPoint);
  connect(clear_draft_btn, &QPushButton::clicked, this, &SearchControlPanel::clearDraft);
  connect(publish_task_btn, &QPushButton::clicked, this, &SearchControlPanel::publishTask);
  connect(clear_task_btn, &QPushButton::clicked, this, &SearchControlPanel::clearTask);

  task_group->setLayout(task_layout);
  root->addWidget(task_group);

  auto* route_group = new QGroupBox("Route control");
  auto* route_layout = new QGridLayout;

  auto* refresh_main_btn = new QPushButton("Refresh Main");
  auto* refresh_backup_btn = new QPushButton("Refresh Backup");
  auto* refresh_all_btn = new QPushButton("Refresh All");
  auto* rf_main_btn = new QPushButton("RF -> Main");
  auto* rf_backup_btn = new QPushButton("RF -> Backup");
  auto* send_route_btn = new QPushButton("Send Route");
  auto* reset_main_btn = new QPushButton("Reset Main");
  auto* reached_btn = new QPushButton("Goal Reached");
  auto* use_b1_btn = new QPushButton("Use B1");
  auto* use_b2_btn = new QPushButton("Use B2");
  auto* use_b3_btn = new QPushButton("Use B3");

  route_layout->addWidget(refresh_main_btn, 0, 0);
  route_layout->addWidget(refresh_backup_btn, 0, 1);
  route_layout->addWidget(refresh_all_btn, 0, 2);
  route_layout->addWidget(rf_main_btn, 1, 0);
  route_layout->addWidget(rf_backup_btn, 1, 1);
  route_layout->addWidget(send_route_btn, 1, 2);
  route_layout->addWidget(reset_main_btn, 2, 0);
  route_layout->addWidget(reached_btn, 2, 1);
  route_layout->addWidget(use_b1_btn, 3, 0);
  route_layout->addWidget(use_b2_btn, 3, 1);
  route_layout->addWidget(use_b3_btn, 3, 2);

  connect(refresh_main_btn, &QPushButton::clicked, this, &SearchControlPanel::refreshMainRoute);
  connect(refresh_backup_btn, &QPushButton::clicked, this, &SearchControlPanel::refreshBackupRoute);
  connect(refresh_all_btn, &QPushButton::clicked, this, &SearchControlPanel::refreshAllRoutes);
  connect(rf_main_btn, &QPushButton::clicked, this, &SearchControlPanel::refreshMainByRf);
  connect(rf_backup_btn, &QPushButton::clicked, this, &SearchControlPanel::refreshBackupByRf);
  connect(send_route_btn, &QPushButton::clicked, this, &SearchControlPanel::sendRouteToUav);
  connect(reset_main_btn, &QPushButton::clicked, this, &SearchControlPanel::resetMainRoute);
  connect(reached_btn, &QPushButton::clicked, this, &SearchControlPanel::goalReached);
  connect(use_b1_btn, &QPushButton::clicked, this, &SearchControlPanel::useB1);
  connect(use_b2_btn, &QPushButton::clicked, this, &SearchControlPanel::useB2);
  connect(use_b3_btn, &QPushButton::clicked, this, &SearchControlPanel::useB3);

  route_group->setLayout(route_layout);
  root->addWidget(route_group);

  auto* state_group = new QGroupBox("State");
  auto* state_layout = new QVBoxLayout;

  mode_label_ = new QLabel;
  hint_label_ = new QLabel;
  hint_label_->setWordWrap(true);
  last_label_ = new QLabel;
  boundary_label_ = new QLabel;
  main_label_ = new QLabel("Main: --");
  backup_label_ = new QLabel("Backup: --");
  rf_label_ = new QLabel("RF: --");
  rf_hint_label_ = new QLabel("RF suggestion: --");
  rf_hint_label_->setWordWrap(true);

  state_layout->addWidget(mode_label_);
  state_layout->addWidget(hint_label_);
  state_layout->addWidget(boundary_label_);
  state_layout->addWidget(main_label_);
  state_layout->addWidget(backup_label_);
  state_layout->addWidget(rf_label_);
  state_layout->addWidget(rf_hint_label_);

  status_text_ = new QPlainTextEdit;
  status_text_->setReadOnly(true);
  status_text_->setMaximumBlockCount(80);
  status_text_->setPlaceholderText("Status from task, route and /rf_gradient_status topics");
  state_layout->addWidget(status_text_);

  state_group->setLayout(state_layout);
  root->addWidget(state_group, 1);

  setLayout(root);
}

void SearchControlPanel::setupRos()
{
  ros::NodeHandle pnh("~");
  pnh.param<std::string>("frame_id", frame_id_, "map");

  set_task_pub_ = nh_.advertise<std_msgs::String>("/set_search_task", 1, false);
  clear_task_pub_ = nh_.advertise<std_msgs::Bool>("/clear_search_task", 1, false);
  main_refresh_pub_ = nh_.advertise<std_msgs::Bool>("/main_routes_refresh", 1, false);
  backup_refresh_pub_ = nh_.advertise<std_msgs::Bool>("/backup_routes_refresh", 1, false);
  all_refresh_pub_ = nh_.advertise<std_msgs::Bool>("/search_routes_refresh", 1, false);
  select_backup_pub_ = nh_.advertise<std_msgs::String>("/select_backup_route", 1, false);
  goal_reached_pub_ = nh_.advertise<std_msgs::Bool>("/ordered_goal_reached", 1, false);
  reset_main_pub_ = nh_.advertise<std_msgs::Bool>("/ordered_goal_reset", 1, false);
  draft_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/search_panel_draft_markers", 1, true);
  send_route_pub_ = nh_.advertise<std_msgs::Bool>("/send_route_to_uav", 1, false);

  clicked_point_sub_ = nh_.subscribe("/clicked_point", 20, &SearchControlPanel::clickedPointCallback, this);
  search_task_status_sub_ = nh_.subscribe("/search_task_status", 10, &SearchControlPanel::searchTaskStatusCallback, this);
  ordered_goal_sequence_sub_ = nh_.subscribe("/ordered_goal_sequence", 10, &SearchControlPanel::orderedGoalSequenceCallback, this);
  backup_goal_sequence_sub_ = nh_.subscribe("/backup_goal_sequence", 10, &SearchControlPanel::backupGoalSequenceCallback, this);
  rf_gradient_status_sub_ = nh_.subscribe("/rf_gradient_status", 10, &SearchControlPanel::rfGradientStatusCallback, this);
}

void SearchControlPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);

  QString target_id;
  if (config.mapGetString("TargetID", &target_id))
  {
    target_id_edit_->setText(target_id);
  }

  float sigma = 3.0f;
  if (config.mapGetFloat("Sigma", &sigma))
  {
    sigma_spin_->setValue(static_cast<double>(sigma));
  }

  bool auto_refresh = true;
  if (config.mapGetBool("AutoRefreshAfterPublish", &auto_refresh))
  {
    auto_refresh_check_->setChecked(auto_refresh);
  }
}

void SearchControlPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
  config.mapSetValue("TargetID", target_id_edit_->text());
  config.mapSetValue("Sigma", sigma_spin_->value());
  config.mapSetValue("AutoRefreshAfterPublish", auto_refresh_check_->isChecked());
}

void SearchControlPanel::clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  QMetaObject::invokeMethod(
      this,
      "handleClickedPointQt",
      Qt::QueuedConnection,
      Q_ARG(double, msg->point.x),
      Q_ARG(double, msg->point.y),
      Q_ARG(double, msg->point.z));
}

void SearchControlPanel::searchTaskStatusCallback(const std_msgs::String::ConstPtr& msg)
{
  QMetaObject::invokeMethod(
      this,
      "handleSearchTaskStatusQt",
      Qt::QueuedConnection,
      Q_ARG(QString, QString::fromStdString(msg->data)));
}

void SearchControlPanel::orderedGoalSequenceCallback(const std_msgs::String::ConstPtr& msg)
{
  QMetaObject::invokeMethod(
      this,
      "handleOrderedGoalSequenceQt",
      Qt::QueuedConnection,
      Q_ARG(QString, QString::fromStdString(msg->data)));
}

void SearchControlPanel::backupGoalSequenceCallback(const std_msgs::String::ConstPtr& msg)
{
  QMetaObject::invokeMethod(
      this,
      "handleBackupGoalSequenceQt",
      Qt::QueuedConnection,
      Q_ARG(QString, QString::fromStdString(msg->data)));
}

void SearchControlPanel::rfGradientStatusCallback(const std_msgs::String::ConstPtr& msg)
{
  QMetaObject::invokeMethod(
      this,
      "handleRfGradientStatusQt",
      Qt::QueuedConnection,
      Q_ARG(QString, QString::fromStdString(msg->data)));
}

void SearchControlPanel::setMode(InputMode mode, const QString& hint)
{
  mode_ = mode;
  hint_label_->setText(hint);
  updateLabels();
}

void SearchControlPanel::setLastKnownMode()
{
  setMode(InputMode::CLICK_LAST_KNOWN, "Select the RViz Publish Point tool, then click the last known position.");
}

void SearchControlPanel::setRectBoundaryMode()
{
  rect_points_.clear();
  boundary_type_ = BoundaryType::NONE;
  setMode(InputMode::CLICK_RECT_BOUNDARY, "Select Publish Point and click two opposite corners of the rectangular search boundary.");
  publishDraftMarkers();
}

void SearchControlPanel::setPolyBoundaryMode()
{
  poly_points_.clear();
  boundary_type_ = BoundaryType::NONE;
  setMode(InputMode::CLICK_POLY_BOUNDARY, "Select Publish Point and click polygon vertices. Press Finish Poly when done.");
  publishDraftMarkers();
}

void SearchControlPanel::finishPolyBoundary()
{
  if (poly_points_.size() < 3)
  {
    hint_label_->setText("Polygon boundary needs at least 3 points.");
    updateLabels();
    return;
  }

  boundary_type_ = BoundaryType::POLYGON;
  setMode(InputMode::IDLE, "Polygon boundary is ready. Press Publish Task to send it.");
  publishDraftMarkers();
}

void SearchControlPanel::undoPoint()
{
  if (mode_ == InputMode::CLICK_POLY_BOUNDARY && !poly_points_.empty())
  {
    poly_points_.pop_back();
    hint_label_->setText("Removed last polygon point.");
  }
  else if (mode_ == InputMode::CLICK_RECT_BOUNDARY && !rect_points_.empty())
  {
    rect_points_.pop_back();
    hint_label_->setText("Removed last rectangle corner.");
  }
  else if (boundary_type_ == BoundaryType::POLYGON && !poly_points_.empty())
  {
    poly_points_.pop_back();
    if (poly_points_.size() < 3)
    {
      boundary_type_ = BoundaryType::NONE;
    }
    hint_label_->setText("Removed last polygon point.");
  }
  else if (boundary_type_ == BoundaryType::RECT && !rect_points_.empty())
  {
    rect_points_.pop_back();
    boundary_type_ = BoundaryType::NONE;
    hint_label_->setText("Cleared rectangle boundary.");
  }
  else if (has_last_known_)
  {
    has_last_known_ = false;
    hint_label_->setText("Cleared last known point.");
  }
  else
  {
    hint_label_->setText("No draft point to undo.");
  }

  updateLabels();
  publishDraftMarkers();
}

void SearchControlPanel::clearDraft()
{
  mode_ = InputMode::IDLE;
  has_last_known_ = false;
  boundary_type_ = BoundaryType::NONE;
  rect_points_.clear();
  poly_points_.clear();
  hint_label_->setText("Draft cleared.");
  updateLabels();
  publishDraftMarkers();
}

void SearchControlPanel::clearTask()
{
  publishBool(clear_task_pub_, true);
  clearDraft();
  hint_label_->setText("Clear task command published.");
}

void SearchControlPanel::handleClickedPointQt(double x, double y, double z)
{
  Point3 p;
  p.x = x;
  p.y = y;
  p.z = z;

  if (mode_ == InputMode::CLICK_LAST_KNOWN)
  {
    last_known_ = p;
    has_last_known_ = true;
    setMode(InputMode::IDLE, "Last known point recorded. Now set a boundary or publish the task.");
  }
  else if (mode_ == InputMode::CLICK_RECT_BOUNDARY)
  {
    rect_points_.push_back(p);
    if (rect_points_.size() >= 2)
    {
      rect_points_.resize(2);
      boundary_type_ = BoundaryType::RECT;
      setMode(InputMode::IDLE, "Rectangle boundary recorded. Press Publish Task to send it.");
    }
    else
    {
      hint_label_->setText("First rectangle corner recorded. Click the opposite corner.");
    }
  }
  else if (mode_ == InputMode::CLICK_POLY_BOUNDARY)
  {
    poly_points_.push_back(p);
    hint_label_->setText(QString("Polygon point %1 recorded. Continue clicking or press Finish Poly.").arg(poly_points_.size()));
  }
  else
  {
    hint_label_->setText("Clicked point received, but no input mode is active. Press Rect Region or Poly Region first.");
  }

  updateLabels();
  publishDraftMarkers();
}

void SearchControlPanel::publishTask()
{
  // Region-only mode: the boundary itself is the last-seen area.
  // No last-known point is required.

  if (boundary_type_ == BoundaryType::NONE)
  {
    hint_label_->setText("Cannot publish task: search region boundary is missing.");
    updateLabels();
    return;
  }

  std_msgs::String msg;
  msg.data = buildTaskJson();
  set_task_pub_.publish(msg);

  if (auto_refresh_check_->isChecked())
  {
    publishBool(all_refresh_pub_, true);
  }

  hint_label_->setText("Region task published to /set_search_task.");
  status_text_->appendPlainText(QString("[Publish Task]\n%1").arg(QString::fromStdString(msg.data)));
}

void SearchControlPanel::refreshMainRoute()
{
  publishBool(main_refresh_pub_, true);
  hint_label_->setText("Main route refresh command published.");
}

void SearchControlPanel::refreshBackupRoute()
{
  publishBool(backup_refresh_pub_, true);
  hint_label_->setText("Backup route refresh command published.");
}

void SearchControlPanel::refreshMainByRf()
{
  publishBool(main_refresh_pub_, true);
  hint_label_->setText("RF-guided main route refresh published. Check RF mode and route result.");
}

void SearchControlPanel::refreshBackupByRf()
{
  publishBool(backup_refresh_pub_, true);
  hint_label_->setText("RF-guided backup route refresh published. Use this when RF mode suggests BACKUP_BRANCH.");
}

void SearchControlPanel::sendRouteToUav()
{
  publishBool(send_route_pub_, true);
  hint_label_->setText("Send route command published to /send_route_to_uav.");
}

void SearchControlPanel::refreshAllRoutes()
{
  publishBool(all_refresh_pub_, true);
  hint_label_->setText("Combined route refresh command published.");
}

void SearchControlPanel::resetMainRoute()
{
  publishBool(reset_main_pub_, true);
  hint_label_->setText("Reset main route command published.");
}

void SearchControlPanel::goalReached()
{
  publishBool(goal_reached_pub_, true);
  hint_label_->setText("Goal reached command published.");
}

void SearchControlPanel::useB1()
{
  selectBackupRoute("B1");
}

void SearchControlPanel::useB2()
{
  selectBackupRoute("B2");
}

void SearchControlPanel::useB3()
{
  selectBackupRoute("B3");
}

void SearchControlPanel::publishBool(ros::Publisher& pub, bool value)
{
  std_msgs::Bool msg;
  msg.data = value;
  pub.publish(msg);
}

void SearchControlPanel::selectBackupRoute(const std::string& route_id)
{
  std_msgs::String msg;
  msg.data = route_id;
  select_backup_pub_.publish(msg);
  hint_label_->setText(QString("Selected backup route %1 as main route.").arg(QString::fromStdString(route_id)));
}

std::string SearchControlPanel::jsonEscape(const std::string& s) const
{
  std::ostringstream out;
  for (char c : s)
  {
    switch (c)
    {
      case '"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\b': out << "\\b"; break;
      case '\f': out << "\\f"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20)
        {
          out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(c);
        }
        else
        {
          out << c;
        }
    }
  }
  return out.str();
}

std::string SearchControlPanel::buildTaskJson() const
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(3);

  ss << "{";
  ss << "\"target_id\":\"" << jsonEscape(target_id_edit_->text().toStdString()) << "\",";
  // Do not publish last_known. The task boundary itself is the last-seen/search region.
  ss << "\"boundary\":";

  if (boundary_type_ == BoundaryType::RECT && rect_points_.size() >= 2)
  {
    const double xmin = std::min(rect_points_[0].x, rect_points_[1].x);
    const double xmax = std::max(rect_points_[0].x, rect_points_[1].x);
    const double ymin = std::min(rect_points_[0].y, rect_points_[1].y);
    const double ymax = std::max(rect_points_[0].y, rect_points_[1].y);
    ss << "{";
    ss << "\"type\":\"rect\",";
    ss << "\"xmin\":" << xmin << ",";
    ss << "\"ymin\":" << ymin << ",";
    ss << "\"xmax\":" << xmax << ",";
    ss << "\"ymax\":" << ymax;
    ss << "}";
  }
  else
  {
    ss << "{";
    ss << "\"type\":\"polygon\",";
    ss << "\"points\":[";
    for (size_t i = 0; i < poly_points_.size(); ++i)
    {
      if (i > 0)
      {
        ss << ",";
      }
      ss << "[" << poly_points_[i].x << "," << poly_points_[i].y << "]";
    }
    ss << "]";
    ss << "}";
  }

  ss << ",\"enforce_boundary\":true";
  ss << "}";
  return ss.str();
}

QString SearchControlPanel::pointSummary(const Point3& p) const
{
  return QString("(%1, %2, %3)")
      .arg(p.x, 0, 'f', 2)
      .arg(p.y, 0, 'f', 2)
      .arg(p.z, 0, 'f', 2);
}

QString SearchControlPanel::boundarySummary() const
{
  if (boundary_type_ == BoundaryType::RECT && rect_points_.size() >= 2)
  {
    return QString("rect: %1 -> %2").arg(pointSummary(rect_points_[0]), pointSummary(rect_points_[1]));
  }

  if (boundary_type_ == BoundaryType::POLYGON && poly_points_.size() >= 3)
  {
    return QString("polygon: %1 points").arg(poly_points_.size());
  }

  if (mode_ == InputMode::CLICK_RECT_BOUNDARY)
  {
    return QString("rect draft: %1/2 points").arg(rect_points_.size());
  }

  if (mode_ == InputMode::CLICK_POLY_BOUNDARY)
  {
    return QString("polygon draft: %1 points").arg(poly_points_.size());
  }

  return "NONE";
}

void SearchControlPanel::updateLabels()
{
  QString mode_text = "IDLE";
  if (mode_ == InputMode::CLICK_LAST_KNOWN)
  {
    mode_text = "CLICK_LAST_KNOWN";
  }
  else if (mode_ == InputMode::CLICK_RECT_BOUNDARY)
  {
    mode_text = "CLICK_RECT_BOUNDARY";
  }
  else if (mode_ == InputMode::CLICK_POLY_BOUNDARY)
  {
    mode_text = "CLICK_POLY_BOUNDARY";
  }

  mode_label_->setText(QString("Mode: %1").arg(mode_text));
  boundary_label_->setText(QString("Draft Region: %1").arg(boundarySummary()));
}

QString SearchControlPanel::shortJsonSummary(const QString& text) const
{
  QString summary = text;
  summary.replace("\n", " ");
  summary.replace("\r", " ");
  if (summary.length() > 520)
  {
    summary = summary.left(520) + "...";
  }
  return summary;
}

void SearchControlPanel::handleSearchTaskStatusQt(QString text)
{
  status_text_->appendPlainText(QString("[Task Status]\n%1").arg(shortJsonSummary(text)));
}

void SearchControlPanel::handleOrderedGoalSequenceQt(QString text)
{
  QRegularExpression num_re("\"num_goals\"\\s*:\\s*(\\d+)");
  QRegularExpression action_re("\"queue_action\"\\s*:\\s*\"([^\"]+)\"");
  QRegularExpressionMatch num_match = num_re.match(text);
  QRegularExpressionMatch action_match = action_re.match(text);

  QString s = "Main: ";
  s += num_match.hasMatch() ? QString("%1 goals").arg(num_match.captured(1)) : "received";
  if (action_match.hasMatch())
  {
    s += QString(", %1").arg(action_match.captured(1));
  }

  main_label_->setText(s);
  status_text_->appendPlainText(QString("[Main Route]\n%1").arg(shortJsonSummary(text)));
}

void SearchControlPanel::handleBackupGoalSequenceQt(QString text)
{
  QRegularExpression count_re("\"num_routes\"\\s*:\\s*(\\d+)");
  QRegularExpression routes_re("\"route_id\"\\s*:\\s*\"([^\"]+)\"");
  QRegularExpressionMatch count_match = count_re.match(text);

  QStringList ids;
  QRegularExpressionMatchIterator it = routes_re.globalMatch(text);
  while (it.hasNext())
  {
    ids << it.next().captured(1);
  }
  ids.removeDuplicates();

  QString s = "Backup: ";
  if (count_match.hasMatch())
  {
    s += QString("%1 routes").arg(count_match.captured(1));
  }
  else if (!ids.isEmpty())
  {
    s += QString("%1 routes").arg(ids.size());
  }
  else
  {
    s += "received";
  }

  if (!ids.isEmpty())
  {
    s += QString(" [%1]").arg(ids.join(", "));
  }

  backup_label_->setText(s);
  status_text_->appendPlainText(QString("[Backup Route]\n%1").arg(shortJsonSummary(text)));
}


void SearchControlPanel::handleRfGradientStatusQt(QString text)
{
  QRegularExpression mode_re("\\\"mode\\\"\\s*:\\s*\\\"([^\\\"]+)\\\"");
  QRegularExpression conf_re("\\\"confidence\\\"\\s*:\\s*([-+0-9.eE]+)");
  QRegularExpression trend_re("\\\"trend\\\"\\s*:\\s*([-+0-9.eE]+)");
  QRegularExpression heading_re("\\\"heading_deg\\\"\\s*:\\s*([-+0-9.eE]+)");
  QRegularExpression suggestion_re("\\\"suggestion\\\"\\s*:\\s*\\\"([^\\\"]+)\\\"");
  QRegularExpression count_re("\\\"sample_count\\\"\\s*:\\s*(\\d+)");

  QRegularExpressionMatch mode_match = mode_re.match(text);
  QRegularExpressionMatch conf_match = conf_re.match(text);
  QRegularExpressionMatch trend_match = trend_re.match(text);
  QRegularExpressionMatch heading_match = heading_re.match(text);
  QRegularExpressionMatch suggestion_match = suggestion_re.match(text);
  QRegularExpressionMatch count_match = count_re.match(text);

  QString mode = mode_match.hasMatch() ? mode_match.captured(1) : "UNKNOWN";
  QString conf = conf_match.hasMatch() ? conf_match.captured(1) : "--";
  QString trend = trend_match.hasMatch() ? trend_match.captured(1) : "--";
  QString heading = heading_match.hasMatch() ? heading_match.captured(1) : "--";
  QString count = count_match.hasMatch() ? count_match.captured(1) : "--";
  QString suggestion = suggestion_match.hasMatch() ? suggestion_match.captured(1) : "--";

  rf_label_->setText(QString("RF: %1, conf=%2, trend=%3, heading=%4 deg, samples=%5")
                         .arg(mode, conf, trend, heading, count));
  rf_hint_label_->setText(QString("RF suggestion: %1").arg(suggestion));
  status_text_->appendPlainText(QString("[RF Gradient]\n%1").arg(shortJsonSummary(text)));
}

visualization_msgs::Marker SearchControlPanel::makeDeleteAllMarker() const
{
  visualization_msgs::Marker m;
  m.header.frame_id = frame_id_;
  m.header.stamp = ros::Time::now();
  m.action = visualization_msgs::Marker::DELETEALL;
  return m;
}

visualization_msgs::Marker SearchControlPanel::makeSphereMarker(
    int id,
    const std::string& ns,
    const Point3& p,
    double scale,
    float r,
    float g,
    float b,
    float a) const
{
  visualization_msgs::Marker m;
  m.header.frame_id = frame_id_;
  m.header.stamp = ros::Time::now();
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::Marker::SPHERE;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.position.x = p.x;
  m.pose.position.y = p.y;
  m.pose.position.z = p.z + 0.20;
  m.pose.orientation.w = 1.0;
  m.scale.x = scale;
  m.scale.y = scale;
  m.scale.z = scale;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = a;
  m.lifetime = ros::Duration(0.0);
  return m;
}

visualization_msgs::Marker SearchControlPanel::makeTextMarker(
    int id,
    const std::string& ns,
    const Point3& p,
    const std::string& text,
    double scale,
    float r,
    float g,
    float b,
    float a) const
{
  visualization_msgs::Marker m;
  m.header.frame_id = frame_id_;
  m.header.stamp = ros::Time::now();
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.position.x = p.x;
  m.pose.position.y = p.y;
  m.pose.position.z = p.z + 0.85;
  m.pose.orientation.w = 1.0;
  m.scale.z = scale;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = a;
  m.text = text;
  m.lifetime = ros::Duration(0.0);
  return m;
}

visualization_msgs::Marker SearchControlPanel::makeLineStripMarker(
    int id,
    const std::string& ns,
    const std::vector<Point3>& points,
    bool close_loop,
    double width,
    float r,
    float g,
    float b,
    float a) const
{
  visualization_msgs::Marker m;
  m.header.frame_id = frame_id_;
  m.header.stamp = ros::Time::now();
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::Marker::LINE_STRIP;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = width;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = a;
  m.lifetime = ros::Duration(0.0);

  for (const auto& pt : points)
  {
    geometry_msgs::Point p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = pt.z + 0.10;
    m.points.push_back(p);
  }

  if (close_loop && points.size() >= 2)
  {
    geometry_msgs::Point p;
    p.x = points.front().x;
    p.y = points.front().y;
    p.z = points.front().z + 0.10;
    m.points.push_back(p);
  }

  return m;
}

void SearchControlPanel::publishDraftMarkers()
{
  visualization_msgs::MarkerArray arr;
  arr.markers.push_back(makeDeleteAllMarker());

  int id = 1;

  if (has_last_known_)
  {
    arr.markers.push_back(makeSphereMarker(id++, "draft_last_known", last_known_, 0.45, 1.0f, 0.1f, 0.1f, 1.0f));
    arr.markers.push_back(makeTextMarker(
        id++,
        "draft_last_known_text",
        last_known_,
        "Draft Last\n" + target_id_edit_->text().toStdString(),
        0.45,
        1.0f,
        1.0f,
        1.0f,
        1.0f));
  }

  if (boundary_type_ == BoundaryType::RECT && rect_points_.size() >= 2)
  {
    const double xmin = std::min(rect_points_[0].x, rect_points_[1].x);
    const double xmax = std::max(rect_points_[0].x, rect_points_[1].x);
    const double ymin = std::min(rect_points_[0].y, rect_points_[1].y);
    const double ymax = std::max(rect_points_[0].y, rect_points_[1].y);
    const double z = 0.05;
    std::vector<Point3> pts = {
        {xmin, ymin, z},
        {xmax, ymin, z},
        {xmax, ymax, z},
        {xmin, ymax, z}};
    arr.markers.push_back(makeLineStripMarker(id++, "draft_boundary", pts, true, 0.08, 0.0f, 1.0f, 0.2f, 1.0f));
  }
  else if (boundary_type_ == BoundaryType::POLYGON && poly_points_.size() >= 3)
  {
    arr.markers.push_back(makeLineStripMarker(id++, "draft_boundary", poly_points_, true, 0.08, 0.0f, 1.0f, 0.2f, 1.0f));
  }
  else
  {
    if (mode_ == InputMode::CLICK_RECT_BOUNDARY && !rect_points_.empty())
    {
      for (const auto& pt : rect_points_)
      {
        arr.markers.push_back(makeSphereMarker(id++, "draft_rect_points", pt, 0.28, 0.0f, 0.8f, 1.0f, 1.0f));
      }
    }
    if (mode_ == InputMode::CLICK_POLY_BOUNDARY && !poly_points_.empty())
    {
      arr.markers.push_back(makeLineStripMarker(id++, "draft_poly_points", poly_points_, false, 0.06, 0.0f, 0.8f, 1.0f, 1.0f));
      for (const auto& pt : poly_points_)
      {
        arr.markers.push_back(makeSphereMarker(id++, "draft_poly_vertex", pt, 0.24, 0.0f, 0.8f, 1.0f, 1.0f));
      }
    }
  }

  draft_marker_pub_.publish(arr);
}

void SearchControlPanel::clearDraftMarkers()
{
  visualization_msgs::MarkerArray arr;
  arr.markers.push_back(makeDeleteAllMarker());
  draft_marker_pub_.publish(arr);
}

}  // namespace dk_search_rviz_panel

PLUGINLIB_EXPORT_CLASS(dk_search_rviz_panel::SearchControlPanel, rviz::Panel)
