#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>

class UARTNode : public rclcpp::Node {
public:
  UARTNode() : Node("uart_node"), fd_(-1) {
    RCLCPP_INFO(this->get_logger(), "UARTノード起動");

    // UARTポートを開く
    fd_ = open("/dev/ttyTHS1", O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "UARTポートのオープンに失敗");
      return;
    }
    tcflush(fd_, TCIFLUSH);  // ← これを追加(delete bafa)
    configure_uart();

    // タイマーを1秒周期で起動
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&UARTNode::send_and_receive, this)
    );
  }

  ~UARTNode() {
    if (fd_ >= 0) {
      close(fd_);
    }
  }

private:
  int fd_;
  rclcpp::TimerBase::SharedPtr timer_;

  void configure_uart() {
    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "UART設定読み込み失敗");
      return;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "UART設定失敗");
    }
  }

  void send_and_receive() {
    if (fd_ < 0) return;

    std::string msg = "on\n";
    write(fd_, msg.c_str(), msg.size());
    RCLCPP_INFO(this->get_logger(), "送信: %s", msg.c_str());

    char buf[100];
    int n = read(fd_, buf, sizeof(buf) - 1);
    if (n > 0) {
      buf[n] = '\0';
      RCLCPP_INFO(this->get_logger(), "受信: %s", buf);
      tcflush(fd_, TCIFLUSH);
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UARTNode>();
  rclcpp::spin(node);  // タイマーが動作する間、スピンを継続
  rclcpp::shutdown();
  return 0;
}