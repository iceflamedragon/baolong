#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QLabel>
#include <QMainWindow>
#include <QMutex>
#include <QThread>
#include <QVBoxLayout>

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

  void updateParams(const QString &params);

private:
  QLabel *m_label;
  QMutex m_mutex;
};

#endif // MAINWINDOW_H