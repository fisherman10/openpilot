#pragma once

#include <QButtonGroup>
#include <QFileSystemWatcher>
#include <QFrame>
#include <QLabel>
#include <QPushButton>
#include <QStackedWidget>
#include <QWidget>


#include "selfdrive/ui/qt/widgets/controls.h"

class BranchControl : public ButtonControl {
  Q_OBJECT

public:
  BranchControl();

  void refresh();

private:
  Params params;
  QLabel branch_label;

  void switchToBranch(const QString &branch);
};

// ********** settings window + top-level panels **********

class DevicePanel : public QWidget {
  Q_OBJECT
public:
  explicit DevicePanel(QWidget* parent = nullptr);
signals:
  void showDriverView();
};

class TogglesPanel : public QWidget {
  Q_OBJECT
public:
  explicit TogglesPanel(QWidget *parent = nullptr);
};

class SoftwarePanel : public QWidget {
  Q_OBJECT
public:
  explicit SoftwarePanel(QWidget* parent = nullptr);

private:
  void showEvent(QShowEvent *event) override;
  void updateLabels();

  BranchControl *branchControl;
  LabelControl *gitCommitLbl;
  ButtonControl *osVersionLbl;
  LabelControl *versionLbl;
  LabelControl *lastUpdateLbl;
  ButtonControl *updateBtn;
  ButtonControl *testBtn;

  int dev_tab_counter = 0;

  Params params;
  QFileSystemWatcher *fs_watch;
};

class SettingsWindow : public QFrame {
  Q_OBJECT

public:
  explicit SettingsWindow(QWidget *parent = 0);

protected:
  void hideEvent(QHideEvent *event) override;
  void showEvent(QShowEvent *event) override;

signals:
  void closeSettings();
  void offroadTransition(bool offroad);
  void showDriverView();

private:
  QPushButton *sidebar_alert_widget;
  QWidget *sidebar_widget;
  QButtonGroup *nav_btns;
  QStackedWidget *panel_widget;
  QWidget *panel_header;
  QLabel *header_label;
};
