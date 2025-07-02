## Abgabe für PROLAB Christoph Roth
Dieses package erweitert das example_package das uns zur verfügung gestellt wurde um 3 filter nodes KF EKF und PF.

## 1. Launch
In den Catkin workspace navigieren und das paket mit `catkin_make` bauen und danach mit der launch file starten
```bash
cd catkin_ws
catkin_make
roslaunch example_package start.launch
```

## 2. Erklärung des Programms
Es werden 3 seperate Nodes im launchfile gestartet
  filter_node_KF
  filter_node_EKF
  filter_node_particle
Die ersten beiden publishen jeweils eine `PoseWithCovarianceStamped` und publishen jeweils zu den topics `prediction_KF` bzw. `prediction_EKF`.
Die Particle filter node published ein `PoseArray` auf das topic `prediction_particle`.

In Rviz können jeweils die die einzelnen ergebnisse der berechnungen visulaisiert werden indem das jeweilige topic links im baum ausgewählt wird.
Eventuell muss die visualisierung der `PoseWithCovarianceStamped` vorher noch aktiviert werden mit setzten des hakens. ebenso kann die visualiserung des `PoseArray` deaktiviert werden mit abwählen des hakens.

Es wird auch ein map server gestartet um die aufgezeichnete map in Rviz visualisieren zu können. Die map daten werden für die berechnung der prediciotns nicht verwendet.
Allgemein für die berechnung aller predictions und corrections werden keine `/map` oder `/scan` danten verwendet. Es werden in miener implementierung nur `/odom` und `/imu` messwerte verwendet für die berechungen (keine positionsdaten aus `/odom`).

Um den Roboter zu bewegen wird XTerm verwendet um mit dem teleop den roboter command veleocities (linear und angular) zu übergeben. (Dies wurde aus dem example_package 1:1 übernommen)

## 3. Ergebnisse
Die ergebnissbilder sind im ordner `example_package/pictures | plots` und werden auch im dokumentations paper besprochen.

Allgemein sind meine berechnungen orienteirt an den in den folien gezeigten algorythmen bzw dem buch Probabilistic Robotics von S. Thrun, W. Burgard, and D. Fox.

Trozt allem habe ich es nicht geschafft die kovarianzen richtig zu berechnen / visualisieren. Die kovarianzen sollten ovalförmig sein (bei KF und EKF) allerdings sind sie rund (symmetrisch). Des wieteren wächst die kovarianz beim EKF nur sehr langsam, was nicht zwingend falsch ist da die prediction ziehmlich gut funktionert, die prediction wird ungenau sobald eine vollbremsung mit dem roboter hingelegt wird (von max geschwindigkeit auf 0). Sonst funktionieren die filter so wie sie sollten. Man kann gut sehen wie der KF lineare predictions gut schafft und sobald kurven gefahren werden wird die prediction schlecht. Dieses verhalten zeigt eine deutliche verbesserung im EKF.

Der partikel filter funktioniert und im dynamic reconfigure können die 4 alpha werte angepasst werden. Das resampling hat ienen extra schritt der nicht unbedingt "richtig" ist, und zwar das einzelne partikel die weit weg von allen anderen partikeln sind eleminiert werden im resample schritt. Bei meiner implementation des PF werden beim updaten der weight nur `/odom` und `/imu` daten verwenden (keine positionsdaten aus `/odom`). Da keine `/map` und `/scan` daten verwendet werden ist das resampling nicht besonders effektiv und daher wächst die gesamt verteilung je weiter man sich mit dem roboter bewegt. Es werden 300 partikel verwendet diese anzahl kann im launchfile verändert werden.

## 4. Commands um pose_plot_accumulate.py zu verwenden

Achtung output pfad `_save_dir` anpassen!!

Für KF:
```bash
rosrun example_package pose_plot_accumulate.py _mode:=kf _duration:=60 _save_dir:=/home/christoph/plots _output_file:=kf_plot.png
```
Für EKF:
```bash
rosrun example_package pose_plot_accumulate.py _mode:=ekf _duration:=60 _save_dir:=/home/christoph/plots _output_file:=ekf_plot.png
```
Für PF:
```bash
rosrun example_package pose_plot_accumulate.py _mode:=pf _duration:=60 _save_dir:=/home/christoph/plots _output_file:=pf_plot.png
```
