# oxebots_observers

Pacote de percepção e fusão de dados da equipe **OxeBots** (UFBA) para a categoria RoboCup Small Size League – Entry Level (SSL-EL).

O `oxebots_observers` consome os pacotes brutos das câmeras SSL-Vision e do Game Controller transmitidos via rede pela ponte `ssl_ros_bridge`. Ele é responsável pela validação e filtragem por confiança, fusão espacial de múltiplas câmeras com deduplicação, expiração temporal de detecções e geração do mapa de ocupação (`nav_msgs/OccupancyGrid`) para navegação.

---

## 1. Nós do Pacote

### 1.1 `game_observer_node`
Nó principal de percepção e fusão sensorial.
- **Assinaturas (Subscribers):**
  - `/ssl_vision_bridge/vision_messages` (`ssl_ros_bridge/msg/SSLVisionData`): Pacotes contendo as detecções de todas as câmeras de campo.
  - `/gc_multicast_bridge/referee_messages` (`ssl_ros_bridge/msg/SSLRefereeData`): Mensagens oficiais da partida enviadas pelo árbitro.
- **Publicações (Publishers):**
  - `game_data` (`oxebots_interfaces/msg/GameData`): Estado completo do jogo (bola e robôs de ambos os times), publicado a uma taxa máxima controlada de 60 Hz.
  - `ball_data` (`oxebots_interfaces/msg/BallPosition`): Posição consolidada e confiança da bola.
  - `robot_data` (`oxebots_interfaces/msg/RobotGameData`): Dados de pose de cada robô identificado em campo.
  - `/field_geometry` (`oxebots_interfaces/msg/SSLGeometryData`): Dimensões do campo e posições dos gols e linhas.
  - `map` (`nav_msgs/msg/OccupancyGrid`): Grade de ocupação com raio de inflação ao redor dos robôs para planejadores como D* Lite.

### 1.2 `field_visualizer_node`
Nó de visualização para depuração e exibição gráfica no RViz2.
- **Assinaturas:** `game_data` (`oxebots_interfaces/msg/GameData`).
- **Publicações:** `/field_markers` (`visualization_msgs/msg/MarkerArray`).

---

## 2. Parâmetros (`game_observer_node`)

| Parâmetro | Tipo | Padrão | Descrição |
| :--- | :--- | :--- | :--- |
| `is_yellow_team` | bool | `true` | Define se o time controlado joga com a cor amarela (`true`) ou azul (`false`). |
| `invert_sides` | bool | `false` | Inverte o sentido das coordenadas X/Y caso o time defenda o lado oposto. |
| `team_size` | int | `3` | Quantidade de robôs por time em campo (padrão SSL-EL: 3). |
| `map_resolution` | double | `0.05` | Resolução da grade de ocupação `/map` em metros por célula (5 cm). |
| `robot_inflation_radius`| double | `150.0` | Raio de inflação dos robôs no mapa de custos em milímetros. |
| `topic_retention` | int | `10` | Profundidade da fila de QoS dos tópicos ROS. |
| `vision_topic` | string | `"/ssl_vision_bridge/vision_messages"` | Tópico de entrada dos dados de visão SSL. |
| `referee_topic` | string | `"/gc_multicast_bridge/referee_messages"`| Tópico de entrada dos dados do Game Controller. |
| `publisher_topic` | string | `"game_data"` | Tópico de saída para os dados consolidados do jogo. |
| `min_ball_confidence` | double | `0.3` | Confiança mínima de detecção da visão para aceitar a bola (descarta ruídos). |
| `min_robot_confidence` | double | `0.3` | Confiança mínima de detecção da visão para aceitar robôs. |
| `robot_expiration_ms` | double | `500.0` | Tempo em milissegundos sem detecção para marcar um robô como fora de campo. |
| `ball_expiration_ms` | double | `500.0` | Tempo em milissegundos sem detecção para marcar a bola como perdida. |
| `multi_camera_window_ms`| double | `8.0` | Janela de tempo para reconciliação e fusão de detecções sobrepostas entre câmeras. |

---

## 3. Pré-requisitos e Dependências

- **Sistema:** Ubuntu 24.04 LTS
- **ROS 2:** Jazzy Jalisco
- **Dependências de Pacote:** `rclcpp`, `nav_msgs`, `geometry_msgs`, `visualization_msgs`, `oxebots_interfaces`, `ssl_ros_bridge`.

---

## 4. Como Executar

### 4.1 Execução Direta do Nó
```bash
ros2 run oxebots_observers game_observer_node --ros-args -p is_yellow_team:=false -p invert_sides:=false
```

### 4.2 Execução com Visualizador no RViz
```bash
ros2 run oxebots_observers field_visualizer_node
```

---

## 5. Testes Unitários

O pacote inclui testes automatizados com Google Test cobrindo fusão de câmeras, filtragem de confiança e expiração temporal:
```bash
colcon test --packages-select oxebots_observers
colcon test-result --all --verbose
```

---

## 6. Licença

Este pacote é disponibilizado sob a licença **GPLv3**.
