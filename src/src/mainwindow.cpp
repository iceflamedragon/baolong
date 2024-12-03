#include "../include/mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
  QWidget *centralWidget = new QWidget(this);
  QVBoxLayout *layout = new QVBoxLayout(centralWidget);

  m_label = new QLabel("0", centralWidget);
  layout->addWidget(m_label);

  setCentralWidget(centralWidget);
}

MainWindow::~MainWindow() {}

void MainWindow::updateParams(const QString &params) {
  QMutexLocker locker(&m_mutex);
  m_label->setText(params);
}