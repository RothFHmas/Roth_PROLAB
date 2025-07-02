

Commands um pose_plot_accumulate.py zu verwenden
Achtung output pfad anpassen!!

Für KF:

rosrun example_package pose_plot_accumulate.py _mode:=kf _duration:=60 _save_dir:=/home/christoph/plots _output_file:=kf_plot.png

Für EKF:

rosrun example_package pose_plot_accumulate.py _mode:=ekf _duration:=60 _save_dir:=/home/christoph/plots _output_file:=ekf_plot.png

Für PF:

rosrun example_package pose_plot_accumulate.py _mode:=pf _duration:=60 _save_dir:=/home/christoph/plots _output_file:=pf_plot.png