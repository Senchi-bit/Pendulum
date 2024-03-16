using OxyPlot;
using OxyPlot.Axes;
using OxyPlot.Series;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Threading;

namespace Pendulum
{

    public partial class MainWindow : Window
    {
        private double PendulumMass = 1;
        private double PendulumLength = 1;
        private double DampingCoefficient = 0.1;
        private double Theta0 = 45;
        private double Alpha0 = 0;
        double[] xx = new double[2];
        double time = 0;
        double dt = 0.03;
        Polyline pl = new Polyline();
        double xMin = 0;
        Double yMin = -100;
        double xMax = 50;
        double yMax = 100;
        List<double> pointsX = new List<double>();
        List<double> pointsY = new List<double>();
        List<double> energy = new List<double>();
        List<double> times = new List<double>();


        public MainWindow()
        {
            InitializeComponent();

        }
        private void btnStart_Click(object sender, RoutedEventArgs e)
        {
            pointsX.Clear();
            pointsY.Clear();
            Stop();
            double xMinFixed = 0; // Минимальное значение для оси X
            double xMaxFixed = 50; // Максимальное значение для оси X
            double yMinFixed = -100; // Минимальное значение для оси Y
            double yMaxFixed = 100; // Максимальное значение для оси Y

            var model = new PlotModel { Title = "Theta - Time Curve" };

            model.Axes.Add(new OxyPlot.Axes.LinearAxis { Position = OxyPlot.Axes.AxisPosition.Bottom, Minimum = xMinFixed, Maximum = xMaxFixed });
            model.Axes.Add(new OxyPlot.Axes.LinearAxis { Position = OxyPlot.Axes.AxisPosition.Left, Minimum = yMinFixed, Maximum = yMaxFixed });

            // Построение графика с использованием OxyPlot
            var series = new LineSeries();

            for (int i = 0; i < pointsX.Count; i++)
            {
                series.Points.Add(new DataPoint(pointsX[i], pointsY[i]));
            }

            model.Series.Add(series);
            plotView.Model = model;
            try
            {
                PendulumMass = Convert.ToDouble(tbMass.Text);
                PendulumLength = Convert.ToDouble(tbLength.Text);
                DampingCoefficient = Convert.ToDouble(tbDamping.Text);
                Theta0 = Convert.ToDouble(tbTheta0.Text);
                dt = Convert.ToDouble(tbDt.Text);
            }
            catch {
                PendulumMass = 1;
                PendulumLength = 1; DampingCoefficient = 0.1;
                Theta0 = 45;
                Alpha0 = 0;
                dt = 0.02;
            }
            Theta0 = Math.PI * Theta0 / 180;
            Alpha0 = Math.PI * Alpha0 / 180;

            time = 0;
            xx = new double[2] { Theta0, Alpha0 };
            CompositionTarget.Rendering += StartAnimation;
        }
        private void StartAnimation(object sender, EventArgs e)
        {

            ODESolver.Function[] f = new ODESolver.Function[2] { f1, f2 };
            double[] result = ODESolver.RungeKutta4(f, xx, time, dt);

            double kineticEnergy = 0.5 * PendulumMass * Math.Pow(result[1], 2); // Кинетическая энергия
            double potentialEnergy = PendulumMass * 9.81 * (PendulumLength - PendulumLength * Math.Cos(result[0])); // Потенциальная энергия
            double totalEnergy = kineticEnergy + potentialEnergy; // Полная энергия
            energy.Add(totalEnergy);
            times.Add(time);

            var energyModel = new PlotModel { };

            var energySeries = new LineSeries
            {
                // Установка цвета и толщины линии
                Color = OxyColors.Blue, // Цвет линии
                StrokeThickness = 2 // Толщина линии
            };
            if (energy.Count > 500) {
                energy.RemoveAt(0);
                times.RemoveAt(0);
            }
            for (int i = 0;i < energy.Count; i++)
            {
                energySeries.Points.Add(new DataPoint(times[i], energy[i]));
            }
            energySeries.Points.Add(new DataPoint(time, totalEnergy));
            energyModel.Series.Add(energySeries);
            energyPlotView.Model = energyModel;

            Point pt = new Point(140 + 130 * Math.Sin(result[0]), 20 + 130 * Math.Cos(result[0]));
            ball.Center = pt;
            line1.X2 = pt.X;
            line1.Y2 = pt.Y;

            if (time < xMax)
            {
                pointsX.Add(XNormalize(time) + 10);
                pointsY.Add(YNormalize(180 * result[0] / Math.PI));
                if (pointsX.Count > 1000) {
                    pointsX.RemoveAt(0);
                    pointsY.RemoveAt(0);
                }

            }
            var model = new PlotModel { Title = "Theta - Time Curve" };
            var series = new LineSeries();



            for (int i = 0; i < pointsX.Count; i++)
            {
                series.Points.Add(new DataPoint(pointsX[i], pointsY[i]));
            }
            model.Series.Add(series);
            plotView.Model = model;

            xx = result;
            time += dt;
            if (time > 0 && Math.Abs(result[0]) < 0.01 && Math.Abs(result[1]) < 0.001)
            {
                StopAnim();
            }
        }
        private void btnReset_Click(object sender, RoutedEventArgs e)
        {
            pointsX.Clear();
            pointsY.Clear();

            var model = new PlotModel { Title = "Theta - Time Curve" };

            model.Axes.Add(new OxyPlot.Axes.LinearAxis { Position = OxyPlot.Axes.AxisPosition.Bottom, Minimum = xMin, Maximum = xMax });
            model.Axes.Add(new OxyPlot.Axes.LinearAxis { Position = OxyPlot.Axes.AxisPosition.Left, Minimum = yMin, Maximum = yMax });

            plotView.Model = model;
            PendulumInitialize();
            StopAnim();
        }

        private void energyOxy() { }
        private void StopAnim()
        {

            CompositionTarget.Rendering -= StartAnimation;
        }

        private void PendulumInitialize()
        {
            tbMass.Text = "1";
            tbLength.Text = "1";
            tbDamping.Text = "0,1";
            tbTheta0.Text = "45";
            line1.X2 = 140;
            line1.Y2 = 150;
            ball.Center = new Point(140, 150);
        }
        private void btnStop_Click(object sender, RoutedEventArgs e)
        {
            Stop();
        }
        private void Stop()
        {
            line1.X2 = 140;
            line1.Y2 = 150;
            ball.Center = new Point(140, 150);
            StopAnim();
        }
        private double f1(double[] xx, double t)
        {
            return xx[1];
        }
        private double f2(double[] xx, double t)
        {
            double m = PendulumMass;
            double L = PendulumLength;
            double g = 9.81;
            double b = DampingCoefficient;
            return -g * Math.Sin(xx[0]) / L - b * xx[1] / m;
        }
        private double XNormalize(double x)
        {
            double result = (x - xMin) *
            canvasRight.Width / (xMax - xMin);
            return result;
        }
        private double YNormalize(double y)
        {
            double result = canvasRight.Height - (y - yMin) *
            canvasRight.Height / (yMax - yMin);
            return result;
        }
    }
    
}
