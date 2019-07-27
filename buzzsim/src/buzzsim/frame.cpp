#include <iostream>

#include <ros/package.h>

#include <QPainter>
#include <QTimer>

#include <buzzsim/frame.h>
#include <buzzsim/motion.h>

BuzzsimFrame::BuzzsimFrame(QWidget *parent, Qt::WindowFlags f) : QFrame(parent, f)
{
  setFixedSize(1600, 900);
  setWindowTitle("Buzzsim");

  update_timer_ = new QTimer(this);
  update_timer_->setInterval(16);
  update_timer_->start();

  connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));

  motion::AccelerationLimits acceleration_limits{ 2.0, 2.0 };
  motion::TwistLimits twist_limits{ 5.0, 5.0 };

  motion::Limits limits{ acceleration_limits, twist_limits };
  std::vector<turtle::Turtle::Options> options{
    { "oswin", getTurtleImages()[0], motion::Pose{ { -5.0, 0.0 }, 0.0 }, limits },
    { "kyle", getTurtleImages()[1], motion::Pose{ { 5.0, 0.0 }, 0.0 }, limits }
  };
  world_.init(options);

  update();
}

std::vector<QImage> BuzzsimFrame::getTurtleImages() const
{
  std::vector<QImage> images;

  QString images_path = (ros::package::getPath("igvc_buzzsim") + "/images/").c_str();

  std::vector<QString> names = { "lunar.png", "melodic.png" };
  for (const auto &name : names)
  {
    images.emplace_back(images_path + name);
  }

  return images;
}

void BuzzsimFrame::onUpdate()
{
  world_.update();
  update();
}

BuzzsimFrame::~BuzzsimFrame() = default;

void BuzzsimFrame::paintEvent(QPaintEvent *event)
{
  QPainter painter(this);
  world_.paint(&painter);
}
