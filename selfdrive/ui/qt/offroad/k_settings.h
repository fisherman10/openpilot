#pragma once

#include <QButtonGroup>
#include <QFileSystemWatcher>
#include <QFrame>
#include <QLabel>
#include <QPushButton>
#include <QStackedWidget>
#include <QWidget>

#include "selfdrive/common/features.h"
#include "selfdrive/common/params.h"
#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/qt/widgets/input.h"
#include "selfdrive/common/util.h"

// ********** settings window + top-level panels **********
class SettingsWindow : public QFrame {
  Q_OBJECT

public:
  explicit SettingsWindow(QWidget *parent = 0);

protected:
  void hideEvent(QHideEvent *event) override;
  void showEvent(QShowEvent *event) override;

signals:
  void closeSettings();
  void showDriverView();

private:
  QPushButton *sidebar_alert_widget;
  QWidget *sidebar_widget;
  QButtonGroup *nav_btns;
  QStackedWidget *panel_widget;
};

class DevicePanel : public ListWidget {
  Q_OBJECT
public:
  explicit DevicePanel(SettingsWindow *parent);
signals:
  void showDriverView();

private slots:
  void poweroff();
  void reboot();
  void updateCalibDescription();

private:
  ButtonControl *resetCalibBtn;
  ButtonControl *serialBtn;
  ButtonControl *testBtn;
  ButtonControl *replaceSplashBtn;
  ButtonControl *dumpTmuxBtn;
  SpinboxControl *stopDistanceOffsetSb;
  SpinboxControl *drivePathOffsetSb;
  SpinboxControl *fanPwmOverrideSb;
  SpinboxControl *powerSaverEntryDurationSb;

  int dev_tab_counter = 0;
  Params params;
};

class TogglesPanel : public ListWidget {
  Q_OBJECT
public:
  explicit TogglesPanel(SettingsWindow *parent);
};

class PersonalisedPanel : public ListWidget {
  Q_OBJECT
public:
  explicit PersonalisedPanel(QWidget* parent = nullptr);
private:
  SpinboxControl *stopDistanceOffsetSb;
  SpinboxControl *drivePathOffsetSb;
  SpinboxControl *fanPwmOverrideSb;
  SpinboxControl *powerSaverEntryDurationSb;

};

class FeaturesControl : public ButtonControl {
  Q_OBJECT

public:
  FeaturesControl() : ButtonControl("Features Package", "EDIT", "Warning: Only use under guidance of a support staff.") {
    package_label = new QLabel();
    package_label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    package_label->setStyleSheet("color: #aaaaaa");
    setElidedText(package_label, Params().get("FeaturesPackage").c_str());
    hlayout->insertWidget(1, package_label);
    connect(this, &ButtonControl::clicked, [=] {
      InputDialog dialog("Enter Feature Package Names", this,
        "Feature package names are separated by commas ( \u2009<b>,</b>\u2009 ).<br>Empty to use the default feature package.");
      dialog.setMinLength(0);
      QString currentFeatures = Params().get("FeaturesPackage").c_str();
      if (currentFeatures != "default") dialog.updateDefaultText(currentFeatures + ", ");
      int ret = dialog.exec();
      if (ret == QDialog::Accepted) {
        QString packages = dialog.text();
        int result = Features().set_package(packages.toStdString());
        setElidedText(package_label, Params().get("FeaturesPackage").c_str());
        if (result == -1) {
          ConfirmationDialog::alert("\nSome feature package names\nare invalid and were not added.", this);
        }
      }
    });
  }

private:
  QLabel *package_label;
};

class FixFingerprintSelect : public ButtonControl {
  Q_OBJECT

public:
  FixFingerprintSelect() : ButtonControl("Fix Fingerprint", "SET", "Warning: Selecting the wrong car fingerprint can be dangerous!") {
    selection_label = new QLabel();
    selection_label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    selection_label->setStyleSheet("color: #aaaaaa");
    setElidedText(selection_label, Params().get("FixFingerprint").c_str());
    hlayout->insertWidget(1, selection_label);
    connect(this, &ButtonControl::clicked, [=] {
      QString package = InputDialog::getText("Enter Car Model", this);
      if (package.length() > 0) {
        Params().put("FixFingerprint",package.toStdString());
      }
      else {
        Params().put("FixFingerprint", "");
      }
      setElidedText(selection_label, Params().get("FixFingerprint").c_str());
    });
  }

private:
  QLabel *selection_label;
};

class ChangeBranchSelect : public ButtonControl {
  Q_OBJECT

public:
  // Set upstream for the current branch and ensure it works for local branches
  std::string setUpstream(const std::string& br) {
    std::string b = br;
    std::string fallbackBranch = "snapshot";

    // Check for internet connection to git origin
    std::string checkConnectionCmd = "git ls-remote origin > /dev/null 2>&1";
    int connectionStatus = std::system(checkConnectionCmd.c_str());

    // If the branch is empty, set to fallbackBranch
    if (br.empty()) {
      b = fallbackBranch;
    } else if (connectionStatus == 0) {
      // If we can connect to git origin
      // Command to check if the branch exists in remote
      std::string checkBranchCmd = "git ls-remote --heads origin " + br + " | grep -q refs/heads/" + br;
      bool branchExists = (std::system(checkBranchCmd.c_str()) == 0);

      // If the branch doesn't exist in the remote, set to fallbackBranch
      if (!branchExists) {
        b = fallbackBranch;
      }
    }

    // Ensure the commands run regardless of internet connectivity and overwrite existing upstream
    std::string cmd = "git config remote.origin.fetch '+refs/heads/" + b + ":refs/remotes/origin/" + b + "'";
    cmd += "; git fetch origin '" + b + "'; git branch -u origin/" + b;
    cmd += "; git config branch." + b + ".merge refs/heads/" + b;
    return cmd;
  }

  ChangeBranchSelect() : ButtonControl("Change Branch", "SET", "Warning: Untested branches may cause unexpected behaviours.") {
    selection_label = new QLabel();
    selection_label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    selection_label->setStyleSheet("color: #aaaaaa");
    // Display current branch name
    std::string currentBranchName = util::check_output("git symbolic-ref --short HEAD");
    currentBranchName.pop_back(); // Remove the last character (line feed)
    setElidedText(selection_label, QString::fromStdString(currentBranchName));
    system(setUpstream(currentBranchName).c_str());
    hlayout->insertWidget(1, selection_label);
    connect(this, &ButtonControl::clicked, [=] {
      QString package = InputDialog::getText("Enter Branch Name", this);
      std::string branchName = package.toStdString();
      if (branchName.back() == ' ') {branchName.pop_back();} // Remove extra space if it exists at end of input
      if (branchName.length() > 0) {
        if (branchName == currentBranchName) {
          QString currentPrompt = QString::fromStdString("You are already using the branch\n" + currentBranchName);
          ConfirmationDialog::alert(currentPrompt, this);
        } else if (ConfirmationDialog::confirm("Are you sure to Change Branch?\nAny unsaved changes will be lost.\n\nReboot required, please wait.", this)) {
          // Delete branch, fetch and checkout to the branch. (Ignoring changes not committed/not pushed.)
          std::string changeBranchCommand = "git branch -D " + branchName + "; git fetch origin " + branchName + ":" + branchName + " && git checkout " + branchName + " --force";

          // After change, update config fetch, set upstream (for update), then reboot.
          std::string changeBranchConfigReboot = changeBranchCommand + " && " + setUpstream(branchName) + " && reboot";
          int branchChanged = system(changeBranchConfigReboot.c_str());
		      if (branchChanged != 0) { // If branch not found (error/fatal)
			      QString failedPrompt = QString::fromStdString("Branch " + branchName + " not found.\n\nPlease make sure the branch name is correct and the device is connected to the Internet.");
		        ConfirmationDialog::alert(failedPrompt, this);
          }
        }
      }
    });
  }

private:
  QLabel *selection_label;
};

class SoftwarePanel : public ListWidget {
  Q_OBJECT
public:
  explicit SoftwarePanel(QWidget* parent = nullptr);

private:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;
  void updateLabels();

  LabelControl *gitCommitLbl;
  LabelControl *osVersionLbl;
  LabelControl *versionLbl;
  LabelControl *lastUpdateLbl;
  ButtonControl *updateBtn;
  FixFingerprintSelect *fingerprintInput;
  ChangeBranchSelect *branchInput;
  FeaturesControl *featuresInput;

  Params params;
  QFileSystemWatcher *fs_watch;
  QTimer *timer;
};

class C2NetworkPanel: public ListWidget {
  Q_OBJECT
public:
  explicit C2NetworkPanel(QWidget* parent = nullptr);

private:
  void showEvent(QShowEvent *event) override;
  QString getIPAddress();
  QString getNetworkType();
  LabelControl *ipaddress;
  LabelControl *networkType;
};

