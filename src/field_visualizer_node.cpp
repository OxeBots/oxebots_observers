#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cmath>

class FieldVisualizer : public rclcpp::Node {
public:
    FieldVisualizer() : Node("field_visualizer_node") {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/field_markers", rclcpp::QoS(1).transient_local());
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&FieldVisualizer::publish_field, this));
    }

private:
    void publish_field() {
        visualization_msgs::msg::MarkerArray marker_array;
        
        // Configurações comuns
        double field_length = 4.5;
        double field_width = 3.0;
        double center_circle_radius = 0.5;
        double penalty_area_depth = 0.5;
        double penalty_area_width = 1.35;

        // 1. Linhas Externas e Central
        marker_array.markers.push_back(create_line_marker(0, {
            {-field_length/2, -field_width/2}, {field_length/2, -field_width/2}, // Baixo
            {field_length/2, -field_width/2}, {field_length/2, field_width/2},   // Direita
            {field_length/2, field_width/2}, {-field_length/2, field_width/2},  // Cima
            {-field_length/2, field_width/2}, {-field_length/2, -field_width/2}, // Esquerda
            {0.0, -field_width/2}, {0.0, field_width/2}                         // Central
        }));

        // 2. Círculo Central
        marker_array.markers.push_back(create_circle_marker(1, 0.0, 0.0, center_circle_radius));

        // 3. Área Esquerda
        marker_array.markers.push_back(create_line_marker(2, {
            {-field_length/2, penalty_area_width/2}, {-field_length/2 + penalty_area_depth, penalty_area_width/2},
            {-field_length/2 + penalty_area_depth, penalty_area_width/2}, {-field_length/2 + penalty_area_depth, -penalty_area_width/2},
            {-field_length/2 + penalty_area_depth, -penalty_area_width/2}, {-field_length/2, -penalty_area_width/2}
        }));

        // 4. Área Direita
        marker_array.markers.push_back(create_line_marker(3, {
            {field_length/2, penalty_area_width/2}, {field_length/2 - penalty_area_depth, penalty_area_width/2},
            {field_length/2 - penalty_area_depth, penalty_area_width/2}, {field_length/2 - penalty_area_depth, -penalty_area_width/2},
            {field_length/2 - penalty_area_depth, -penalty_area_width/2}, {field_length/2, -penalty_area_width/2}
        }));

        marker_pub_->publish(marker_array);
    }

    visualization_msgs::msg::Marker create_line_marker(int id, const std::vector<std::pair<double, double>>& points) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "field_lines";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.02; // Espessura da linha
        marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 1.0; marker.color.a = 1.0;

        for (const auto& p : points) {
            geometry_msgs::msg::Point pt;
            pt.x = p.first; pt.y = p.second; pt.z = 0.0;
            marker.points.push_back(pt);
        }
        return marker;
    }

    visualization_msgs::msg::Marker create_circle_marker(int id, double cx, double cy, double r) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.ns = "field_lines";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.scale.x = 0.02;
        marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 1.0; marker.color.a = 1.0;

        for (int i = 0; i <= 40; ++i) {
            double angle = 2.0 * M_PI * i / 40.0;
            geometry_msgs::msg::Point p;
            p.x = cx + r * cos(angle);
            p.y = cy + r * sin(angle);
            marker.points.push_back(p);
        }
        return marker;
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FieldVisualizer>());
    rclcpp::shutdown();
    return 0;
}
