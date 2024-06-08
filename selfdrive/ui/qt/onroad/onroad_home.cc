#include "selfdrive/ui/qt/onroad/onroad_home.h"

#include <QApplication>
#include <QPainter>
#include <QStackedLayout>

#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map_helpers.h"
#include "selfdrive/ui/qt/maps/map_panel.h"
#endif

#include "selfdrive/ui/qt/util.h"

OnroadWindow::OnroadWindow(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *main_layout  = new QVBoxLayout(this);
  main_layout->setMargin(UI_BORDER_SIZE);
  QStackedLayout *stacked_layout = new QStackedLayout;
  stacked_layout->setStackingMode(QStackedLayout::StackAll);
  main_layout->addLayout(stacked_layout);

  nvg = new AnnotatedCameraWidget(VISION_STREAM_ROAD, this);

  QWidget * split_wrapper = new QWidget;
  split = new QHBoxLayout(split_wrapper);
  split->setContentsMargins(0, 0, 0, 0);
  split->setSpacing(0);
  split->addWidget(nvg);

  if (getenv("DUAL_CAMERA_VIEW")) {
    CameraWidget *arCam = new CameraWidget("camerad", VISION_STREAM_ROAD, true, this);
    split->insertWidget(0, arCam);
  }

  if (getenv("MAP_RENDER_VIEW")) {
    CameraWidget *map_render = new CameraWidget("navd", VISION_STREAM_MAP, false, this);
    split->insertWidget(0, map_render);
  }

  stacked_layout->addWidget(split_wrapper);

  alerts = new OnroadAlerts(this);
  alerts->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  stacked_layout->addWidget(alerts);

  // setup stacking order
  alerts->raise();

  setAttribute(Qt::WA_OpaquePaintEvent);
  QObject::connect(uiState(), &UIState::uiUpdate, this, &OnroadWindow::updateState);
  QObject::connect(uiState(), &UIState::offroadTransition, this, &OnroadWindow::offroadTransition);
  QObject::connect(uiState(), &UIState::primeChanged, this, &OnroadWindow::primeChanged);

  QObject::connect(&clickTimer, &QTimer::timeout, this, [this]() {
    clickTimer.stop();
    QMouseEvent *event = new QMouseEvent(QEvent::MouseButtonPress, timeoutPoint, Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::postEvent(this, event);
  });
}

void OnroadWindow::updateState(const UIState &s) {
  if (!s.scene.started) {
    return;
  }

  if (s.scene.map_on_left) {
    split->setDirection(QBoxLayout::LeftToRight);
  } else {
    split->setDirection(QBoxLayout::RightToLeft);
  }

  alerts->updateState(s);
  nvg->updateState(s);

  QColor bgColor = bg_colors[s.status];
  if (bg != bgColor) {
    // repaint border
    bg = bgColor;
    update();
  }
}

void OnroadWindow::mousePressEvent(QMouseEvent* e) {
  // FrogPilot variables
  UIState *s = uiState();
  UIScene &scene = s->scene;

  // FrogPilot clickable widgets
  QSize size = this->size();
  QRect leftRect(0, 0, size.width() / 2, size.height());
  QRect rightRect = leftRect.translated(size.width() / 2, 0);
  bool isLeftSideClicked = leftRect.contains(e->pos()) && scene.speed_limit_changed;
  bool isRightSideClicked = rightRect.contains(e->pos()) && scene.speed_limit_changed;

  QRect maxSpeedRect(7, 25, 225, 225);
  bool isMaxSpeedClicked = maxSpeedRect.contains(e->pos()) && scene.reverse_cruise_ui;

  QRect speedLimitRect(7, 250, 225, 225);
  bool isSpeedLimitClicked = speedLimitRect.contains(e->pos()) && scene.show_slc_offset_ui;

  if (isLeftSideClicked || isRightSideClicked) {
    bool slcConfirmed = isLeftSideClicked && !scene.right_hand_drive || isRightSideClicked && scene.right_hand_drive;
    paramsMemory.putBoolNonBlocking("SLCConfirmed", slcConfirmed);
    paramsMemory.putBoolNonBlocking("SLCConfirmedPressed", true);
    return;
  }

  if (isMaxSpeedClicked) {
    scene.reverse_cruise = !scene.reverse_cruise;
    params.putBoolNonBlocking("ReverseCruise", scene.reverse_cruise);
    updateFrogPilotToggles();
    return;
  }

  if (isSpeedLimitClicked) {
    scene.show_slc_offset = !scene.show_slc_offset;
    params.putBoolNonBlocking("ShowSLCOffset", scene.show_slc_offset);
    return;
  }

  if (scene.experimental_mode_via_screen && e->pos() != timeoutPoint) {
    if (clickTimer.isActive()) {
      clickTimer.stop();

      if (scene.conditional_experimental) {
        int override_value = (scene.conditional_status >= 1 && scene.conditional_status <= 6) ? 0 : (scene.conditional_status >= 7 ? 5 : 6);
        paramsMemory.putIntNonBlocking("CEStatus", override_value);
      } else {
        bool experimentalMode = params.getBool("ExperimentalMode");
        params.putBoolNonBlocking("ExperimentalMode", !experimentalMode);
      }
    } else {
      clickTimer.start(500);
    }
    return;
  }

#ifdef ENABLE_MAPS
  if (map != nullptr) {
    // Switch between map and sidebar when using navigate on openpilot
    bool sidebarVisible = geometry().x() > 0;
    bool show_map = scene.navigate_on_openpilot ? sidebarVisible : !sidebarVisible;
    map->setVisible(show_map && !map->isVisible());
  }
#endif
  // propagation event to parent(HomeWindow)
  QWidget::mousePressEvent(e);
}

void OnroadWindow::createMapWidget() {
#ifdef ENABLE_MAPS
  auto m = new MapPanel(get_mapbox_settings());
  map = m;
  QObject::connect(m, &MapPanel::mapPanelRequested, this, &OnroadWindow::mapPanelRequested);
  QObject::connect(nvg->map_settings_btn, &MapSettingsButton::clicked, m, &MapPanel::toggleMapSettings);
  QObject::connect(nvg->map_settings_btn_bottom, &MapSettingsButton::clicked, m, &MapPanel::toggleMapSettings);
  nvg->map_settings_btn->setEnabled(true);

  m->setFixedWidth(topWidget(this)->width() / 2 - UI_BORDER_SIZE);
  split->insertWidget(0, m);
  // hidden by default, made visible when navRoute is published
  m->setVisible(false);
#endif
}

void OnroadWindow::offroadTransition(bool offroad) {
#ifdef ENABLE_MAPS
  if (!offroad) {
    if (map == nullptr && (uiState()->hasPrime() || !MAPBOX_TOKEN.isEmpty())) {
      createMapWidget();
    }
  }
#endif
  alerts->clear();
}

void OnroadWindow::primeChanged(bool prime) {
#ifdef ENABLE_MAPS
  if (map && (!prime && MAPBOX_TOKEN.isEmpty())) {
    nvg->map_settings_btn->setEnabled(false);
    nvg->map_settings_btn->setVisible(false);
    map->deleteLater();
    map = nullptr;
  } else if (!map && (prime || !MAPBOX_TOKEN.isEmpty())) {
    createMapWidget();
  }
#endif
}

void OnroadWindow::paintEvent(QPaintEvent *event) {
  UIState *s = uiState();
  SubMaster &sm = *(s->sm);

  QPainter p(this);

  // FrogPilot variables
  const UIScene &scene = s->scene;

  bool needsUpdate = false;

  QRect rect = this->rect();
  p.fillRect(rect, QColor(bg.red(), bg.green(), bg.blue(), 255));

  if (scene.show_steering) {
    QLinearGradient gradient(rect.topLeft(), rect.bottomLeft());
    gradient.setColorAt(0.0, bg_colors[STATUS_TRAFFIC_MODE_ACTIVE]);
    gradient.setColorAt(0.15, bg_colors[STATUS_EXPERIMENTAL_MODE_ACTIVE]);
    gradient.setColorAt(0.5, bg_colors[STATUS_CONDITIONAL_OVERRIDDEN]);
    gradient.setColorAt(0.85, bg_colors[STATUS_ENGAGED]);
    gradient.setColorAt(1.0, bg_colors[STATUS_ENGAGED]);

    QBrush brush(gradient);
    int fillWidth = UI_BORDER_SIZE;

    steer = 0.10 * std::abs(scene.steer) + 0.90 * steer;
    int visibleHeight = rect.height() * steer;

    QRect rectToFill, rectToHide;
    if (scene.steering_angle_deg != 0) {
      if (scene.steering_angle_deg < 0) {
        rectToFill = QRect(rect.x(), rect.y() + rect.height() - visibleHeight, fillWidth, visibleHeight);
        rectToHide = QRect(rect.x(), rect.y(), fillWidth, rect.height() - visibleHeight);
      } else {
        rectToFill = QRect(rect.x() + rect.width() - fillWidth, rect.y() + rect.height() - visibleHeight, fillWidth, visibleHeight);
        rectToHide = QRect(rect.x() + rect.width() - fillWidth, rect.y(), fillWidth, rect.height() - visibleHeight);
      }
      p.fillRect(rectToFill, brush);
      p.fillRect(rectToHide, QColor(bg.red(), bg.green(), bg.blue(), 255));
      needsUpdate = true;
    }
  }

  if (scene.show_signal) {
    static int signalFrames = 0;
    QColor signalBorderColor = bg;

    if (scene.turn_signal_left || scene.turn_signal_right) {
      if (sm.frame % 20 == 0 || signalFrames > 0) {
        signalBorderColor = bg_colors[STATUS_CONDITIONAL_OVERRIDDEN];
        signalFrames = (sm.frame % 20 == 0) ? 15 : signalFrames - 1;
      }

      QRect signalRect;
      if (scene.turn_signal_left) {
        signalRect = QRect(rect.x(), rect.y(), rect.width() / 2, rect.height());
      } else if (scene.turn_signal_right) {
        signalRect = QRect(rect.x() + rect.width() / 2, rect.y(), rect.width() / 2, rect.height());
      }

      if (!signalRect.isEmpty()) {
        p.fillRect(signalRect, signalBorderColor);
        needsUpdate = true;
      }
    } else {
      signalFrames = 0;
    }
  }

  if (scene.show_blind_spot) {
    static int blindspotFramesLeft = 0;
    static int blindspotFramesRight = 0;

    auto getBlindspotColor = [&](bool turnSignal, int &frames) {
      if (turnSignal && sm.frame % 10 == 0) {
        frames = 5;
      }
      return (frames-- > 0) ? bg_colors[STATUS_TRAFFIC_MODE_ACTIVE] : bg;
    };

    QRect blindspotRect;
    if (scene.blind_spot_left) {
      blindspotRect = QRect(rect.x(), rect.y(), rect.width() / 2, rect.height());
      p.fillRect(blindspotRect, getBlindspotColor(scene.turn_signal_left, blindspotFramesLeft));
      needsUpdate = true;
    } else {
      blindspotFramesLeft = 0;
    }

    if (scene.blind_spot_right) {
      blindspotRect = QRect(rect.x() + rect.width() / 2, rect.y(), rect.width() / 2, rect.height());
      p.fillRect(blindspotRect, getBlindspotColor(scene.turn_signal_right, blindspotFramesRight));
      needsUpdate = true;
    } else {
      blindspotFramesRight = 0;
    }
  }

  QString logicsDisplayString;
  auto appendJerkInfo = [&](const QString &label, double value, double difference) {
    logicsDisplayString += QString("%1: %2").arg(label).arg(value, 0, 'f', 3);
    if (difference != 0) {
      logicsDisplayString += QString(" (%1%2)").arg(difference > 0 ? "-" : "", 0).arg(difference, 0, 'f', 3);
    }
    logicsDisplayString += " | ";
  };

  if (scene.show_jerk) {
    appendJerkInfo("Acceleration Jerk", scene.acceleration_jerk, scene.acceleration_jerk_difference);
    appendJerkInfo("Speed Jerk", scene.speed_jerk, scene.speed_jerk_difference);
  }

  if (scene.show_tuning) {
    logicsDisplayString += scene.live_valid
        ? QString("Friction: %1 | Lateral Acceleration: %2").arg(scene.friction, 0, 'f', 3).arg(scene.lat_accel, 0, 'f', 3)
        : "Friction: Calculating... | Lateral Acceleration: Calculating...";
  }

  if (logicsDisplayString.endsWith(" | ")) {
    logicsDisplayString.chop(3);
  }

  if (!logicsDisplayString.isEmpty()) {
    p.setFont(InterFont(28, QFont::DemiBold));
    p.setRenderHint(QPainter::TextAntialiasing);
    p.setPen(Qt::white);

    int logicsWidth = p.fontMetrics().horizontalAdvance(logicsDisplayString);
    int logicsX = (rect.width() - logicsWidth) / 2;
    int logicsY = rect.top() + 27;

    QStringList parts = logicsDisplayString.split(" | ");
    int currentX = logicsX;

    for (const QString &part : parts) {
      QStringList subParts = part.split(" ");
      for (int i = 0; i < subParts.size(); ++i) {
        QString text = subParts[i];

        if (text.endsWith(")") && i > 0 && (subParts[i - 1].contains("Acceleration") || subParts[i - 1].contains("Speed"))) {
          QString prefix = subParts[i - 1] + " (";
          p.drawText(currentX, logicsY, prefix);
          currentX += p.fontMetrics().horizontalAdvance(prefix);
          text.chop(1);
          p.setPen(text.contains("-") ? redColor() : Qt::white);
        } else if (text.startsWith("(") && i > 0) {
          p.drawText(currentX, logicsY, " (");
          currentX += p.fontMetrics().horizontalAdvance(" (");
          text = text.mid(1);
          p.setPen(text.contains("-") ? redColor() : Qt::white);
        } else {
          p.setPen(Qt::white);
        }

        p.drawText(currentX, logicsY, text);
        currentX += p.fontMetrics().horizontalAdvance(text + " ");
        needsUpdate = true;
      }
    }
  }

  if (scene.fps_counter) {
    double fps = scene.fps;

    qint64 currentMillis = QDateTime::currentMSecsSinceEpoch();
    static std::queue<std::pair<qint64, double>> fpsQueue;

    static double avgFPS = 0.0;
    static double maxFPS = 0.0;
    static double minFPS = 99.9;

    minFPS = std::min(minFPS, fps);
    maxFPS = std::max(maxFPS, fps);

    fpsQueue.push({currentMillis, fps});

    while (!fpsQueue.empty() && currentMillis - fpsQueue.front().first > 60000) {
      fpsQueue.pop();
    }

    if (!fpsQueue.empty()) {
      double totalFPS = 0.0;
      for (auto tempQueue = fpsQueue; !tempQueue.empty(); tempQueue.pop()) {
        totalFPS += tempQueue.front().second;
      }
      avgFPS = totalFPS / fpsQueue.size();
    }

    QString fpsDisplayString = QString("FPS: %1 (%2) | Min: %3 | Max: %4 | Avg: %5")
        .arg(qRound(fps))
        .arg(paramsMemory.getInt("CameraFPS"))
        .arg(qRound(minFPS))
        .arg(qRound(maxFPS))
        .arg(qRound(avgFPS));

    p.setFont(InterFont(28, QFont::DemiBold));
    p.setRenderHint(QPainter::TextAntialiasing);
    p.setPen(Qt::white);

    int textWidth = p.fontMetrics().horizontalAdvance(fpsDisplayString);
    int xPos = (rect.width() - textWidth) / 2;
    int yPos = rect.bottom() - 5;

    p.drawText(xPos, yPos, fpsDisplayString);
    needsUpdate = true;
  }

  if (needsUpdate) {
    update();
  }
}
