using OxyPlot;
using OxyPlot.Axes;
using OxyPlot.Series;
using ScottPlot.Colormaps;
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
        private double Theta0 = 30;
        private double Alpha0 = 0;
        double[] xx = new double[2];
        double[] xx2 = new double[2];
        double time = 0;
        double time2 = 0;
        double dt = 0.03;
        double dt2 = 0.03;
        Polyline pl = new Polyline();
        double xMin = 0;
        Double yMin = -100;
        double xMax = 50;
        double yMax = 100;
        double period_result = 0;
        double period_result_g = 0;
        double g1 = 9.81;
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
            //Очистка всех графиков
            pointsX.Clear();
            pointsY.Clear();
            energy.Clear();
            times.Clear();
            Stop();
            period_result_g = 0;
            period_result = 0;


            try
            {
                PendulumMass = Convert.ToDouble(tbMass.Text);
                PendulumLength = Convert.ToDouble(tbLength.Text);
                DampingCoefficient = Convert.ToDouble(tbDamping.Text);
                Theta0 = Convert.ToDouble(tbTheta0.Text);
                dt2 = Convert.ToDouble(tbDt.Text);
            }
            catch {
                PendulumMass = 1;
                PendulumLength = 1; DampingCoefficient = 0;
                Theta0 = 45;
                dt = 0.03;
                dt2 = 0.03;
            }
            //Расчет периода
            Theta0 = Math.PI * Theta0 / 180;
            Alpha0 = Math.PI * Alpha0 / 180;
            period_result = 4.0 * Math.Sqrt(PendulumLength / g1) * cei1(Math.Sin(Theta0 / 2.0));
            period_result_g = (2.0 * 3.1415926 * Math.Sqrt(PendulumLength / g1));
            period.Text = period_result.ToString();
            period_guig.Text = period_result_g.ToString();

            //Таймеры
            time = 0;
            time2 = 0;


            xx = new double[2] { Theta0, Alpha0 };
            xx2 = new double[2] { Theta0,Alpha0 };

            CompositionTarget.Rendering += StartAnimation;
            CompositionTarget.Rendering += StartAnimationForGrafics;
        }
        private void StartAnimation(object sender, EventArgs e)
        {
            ODESolver.Function[] f = new ODESolver.Function[2] { functionForxx1, function2 };
            double[] result = ODESolver.RungeKutta4(f, xx, time, dt);

            Point pt = new Point(140 + 130 * Math.Sin(result[0]), 20 + 130 * Math.Cos(result[0]));
            ball.Center = pt;
            line1.X2 = pt.X;
            line1.Y2 = pt.Y;

            xx = result;
            time += dt;

            if (time > 0 && Math.Abs(result[0]) < 0.01 && Math.Abs(result[1]) < 0.001)
            {
                StopAnim();
            }
        }

        private void  StartAnimationForGrafics(object sender, EventArgs e)
        {
            ODESolver.Function[] f = new ODESolver.Function[2] { functionForxx1, function2 };
            double[] result2 = ODESolver.RungeKutta4(f, xx2, time2, dt2);
            //Расчет энергии
            double kineticEnergy = 0.5 * PendulumMass * Math.Pow(result2[1], 2); // Кинетическая энергия
            double potentialEnergy = PendulumMass * 9.81 * (PendulumLength - PendulumLength * Math.Cos(result2[0])); // Потенциальная энергия
            double totalEnergy = kineticEnergy + potentialEnergy; // Полная энергия в Джоулях
            energy.Add(totalEnergy);
            times.Add(time2);

            //Вывод графика энергии
            var energyModel = new PlotModel { };
            var energySeries = new LineSeries
            {
                // Установка цвета и толщины линии
                Color = OxyColors.Blue, // Цвет линии
                StrokeThickness = 2 // Толщина линии
            };
            if (energy.Count > 500)
            {
                energy.RemoveAt(0);
                times.RemoveAt(0);
            }
            for (int i = 0; i < energy.Count; i++)
            {
                energySeries.Points.Add(new DataPoint(times[i], energy[i]));
            }
            energySeries.Points.Add(new DataPoint(time2, totalEnergy));
            energyModel.Series.Add(energySeries);
            energyPlotView.Model = energyModel;
            //______________________________________________________________
            //Вывод графика для координат маятника
            pointsX.Add(time2 + 10);
            pointsY.Add(result2[0] * 180 / Math.PI);
            if (pointsX.Count > 1000)
            {
                pointsX.RemoveAt(0);
                pointsY.RemoveAt(0);
            }
            
            var model = new PlotModel { Title = "Theta - Time" };
            var series = new LineSeries();

            for (int i = 0; i < pointsX.Count; i++)
            {
                series.Points.Add(new DataPoint(pointsX[i], pointsY[i]));
            }
            model.Series.Add(series);
            plotView.Model = model;

            xx2 = result2;
            time2 += dt2;
            if (time > 0 && Math.Abs(result2[0]) < 0.01 && Math.Abs(result2[1]) < 0.001)
            {
                StopAnimChart();
            }

        }
        private void btnReset_Click(object sender, RoutedEventArgs e)
        {
            pointsX.Clear();
            pointsY.Clear();
            energy.Clear();
            times.Clear();
            period_result = 0;
            period_result_g = 0;
            var model = new PlotModel { Title = "Theta - Time" };

            model.Axes.Add(new OxyPlot.Axes.LinearAxis { Position = OxyPlot.Axes.AxisPosition.Bottom, Minimum = xMin, Maximum = xMax });
            model.Axes.Add(new OxyPlot.Axes.LinearAxis { Position = OxyPlot.Axes.AxisPosition.Left, Minimum = yMin, Maximum = yMax });

            plotView.Model = model;
            var energyModel = new PlotModel { };


            energyModel.Axes.Add(new OxyPlot.Axes.LinearAxis { Position = OxyPlot.Axes.AxisPosition.Bottom, Minimum = xMin, Maximum = xMax });
            energyModel.Axes.Add(new OxyPlot.Axes.LinearAxis { Position = OxyPlot.Axes.AxisPosition.Left, Minimum = yMin, Maximum = yMax });
            energyPlotView.Model = energyModel;
            PendulumInitialize();
            StopAnim();
        }
        private void StopAnimChart()
        {
            CompositionTarget.Rendering -= StartAnimationForGrafics;
        }
        private void StopAnim()
        {

            CompositionTarget.Rendering -= StartAnimation;
            CompositionTarget.Rendering -= StartAnimationForGrafics;
        }

        private void PendulumInitialize()
        {
            tbMass.Text = "1";
            tbLength.Text = "1";
            tbDamping.Text = "0,1";
            tbTheta0.Text = "30";
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
            period_result = 0;
            period_result_g = 0;
            StopAnim();
        }
        private double functionForxx1(double[] xx, double t) //Угловая скорость маятника
        {
            return xx[1];
        }
        private double function2(double[] xx, double t) //Угловое ускорения маятника
        {
            double m = PendulumMass;
            double L = PendulumLength;
            double g = 9.81;
            double b = DampingCoefficient;
            return -g * Math.Sin(xx[0]) / L -b * xx[1] / m;
        }
        public double cei1(double k)// выч. полного эллипт. интеграла 1 рода
        {
            double a, b, t;
            t = 1 - k * k;
            a = (((0.01451196212 * t + 0.03742563713) * t
              + 0.03590092383) * t + 0.09666344259) * t + 1.38629436112;
            b = (((0.00441787012 * t + 0.03328355346) * t + 0.06880248576)
              * t + 0.12498593597) * t + 0.5;
            return (a - b * Math.Log(t));
        }
    }

}
