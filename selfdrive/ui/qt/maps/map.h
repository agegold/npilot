#pragma once

#include <optional>

#include <QGeoCoordinate>
#include <QGestureEvent>
#include <QHash>
#include <QHBoxLayout>
#include <QLabel>
#include <QMap>
#include <QMapboxGL>
#include <QMouseEvent>
#include <QOpenGLWidget>
#include <QPixmap>
#include <QPushButton>
#include <QScopedPointer>
#include <QString>
#include <QTextDocument>
#include <QVBoxLayout>
#include <QWheelEvent>

#include "cereal/messaging/messaging.h"
#include "common/params.h"
#include "common/util.h"
#include "selfdrive/ui/ui.h"
#include "selfdrive/ui/qt/widgets/controls.h"

class MapInstructions : public QWidget {
  Q_OBJECT

private:
  QLabel *distance;
  QLabel *primary;
  QLabel *secondary;
  //QLabel *icon_01;
  NetworkImageWidget *icon_01;
  QHBoxLayout *lane_layout;
  bool is_rhd = false;
  std::vector<QLabel *> lane_labels;
  QHash<QString, QPixmap> pixmap_cache;

public:
  MapInstructions(QWidget * parent=nullptr);
  void buildPixmapCache();
  QString getDistance(float d);
  void updateInstructions(cereal::NavInstruction::Reader instruction);
};

class MapETA : public QWidget {
  Q_OBJECT

public:
  MapETA(QWidget * parent=nullptr);
  void updateETA(float seconds, float seconds_typical, float distance);

private:
  void paintEvent(QPaintEvent *event) override;
  void showEvent(QShowEvent *event) override { format_24h = param.getBool("NavSettingTime24h"); }

  bool format_24h = false;
  QTextDocument eta_doc;
  Params param;
};

class MapWindow : public QOpenGLWidget {
  Q_OBJECT

public:
  MapWindow(const QMapboxGLSettings &);
  ~MapWindow();

private:
  void initializeGL() final;
  void paintGL() final;
  void resizeGL(int w, int h) override;

  QMapboxGLSettings m_settings;
  QScopedPointer<QMapboxGL> m_map;
  QMapbox::AnnotationID marker_id = -1;

  void initLayers();

  void mousePressEvent(QMouseEvent *ev) final;
  void mouseDoubleClickEvent(QMouseEvent *ev) final;
  void mouseMoveEvent(QMouseEvent *ev) final;
  void wheelEvent(QWheelEvent *ev) final;
  bool event(QEvent *event) final;
  bool gestureEvent(QGestureEvent *event);
  void pinchTriggered(QPinchGesture *gesture);
  void setError(const QString &err_str);

  bool m_sourceAdded = false;

  bool loaded_once = false;
  bool allow_open = true;

  // Panning
  QPointF m_lastPos;
  int pan_counter = 0;
  int zoom_counter = 0;

  // Position
  std::optional<QMapbox::Coordinate> last_position;
  std::optional<float> last_bearing;
  FirstOrderFilter velocity_filter;
  bool locationd_valid = false;

  QWidget *map_overlay;
  QLabel *error;
  MapInstructions* map_instructions;
  MapETA* map_eta;
  QPushButton *settings_btn;
  QPixmap directions_icon, settings_icon;

  // Blue with normal nav, green when nav is input into the model
  QColor getNavPathColor(bool nav_enabled) {
    return nav_enabled ? QColor("#31ee73") : QColor("#31a1ee");
  }

  void clearRoute();
  void updateDestinationMarker();
  uint64_t route_rcv_frame = 0;

private slots:
  void updateState(const UIState &s);

public slots:
  void offroadTransition(bool offroad);

signals:
  void requestVisible(bool visible);
  void requestSettings(bool settings);
};
