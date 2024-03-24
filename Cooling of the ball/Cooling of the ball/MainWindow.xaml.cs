using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
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


namespace Cooling_of_the_ball
{
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        private const double Radius = 0.03; // Радиус шара в метрах
        private const double ThermalConductivity = 2e-7; // Коэффициент температуропроводности в м²/с
        private const double InitialTemperature = 100.0; // Начальная температура шара в градусах Цельсия
        private const double BoundaryTemperature = 20.0; // Температура на границе шара в градусах Цельсия
        private const int SpatialSteps = 50; // Число шагов по координате
        private const double StepSize = 0.9 * (Radius * Radius) / (2 * ThermalConductivity); // Шаг по времени
        private const int TotalTimeSteps = 500; // Общее количество шагов по времени

        private double[,] temperatureField; // Поле температур
        private double currentTimeStep = 0;

        public event PropertyChangedEventHandler PropertyChanged;

        public MainWindow()
        {
            InitializeComponent();
            InitializeTemperatureField();
            StartCooling();
        }

        private void InitializeTemperatureField()
        {
            temperatureField = new double[SpatialSteps, TotalTimeSteps + 1];
            for (int i = 0; i < SpatialSteps; i++)
            {
                temperatureField[i, 0] = InitialTemperature;
            }
        }

        private async void StartCooling()
        {
            await Task.Run(async () =>
            {
                for (int t = 1; t <= TotalTimeSteps; t++)
                {
                    for (int i = 0; i < SpatialSteps; i++)
                    {
                        double alpha = ThermalConductivity * StepSize / (Radius * Radius);
                        double prevTemp = (i == 0) ? 0 : temperatureField[i - 1, t - 1];
                        double nextTemp = (i == SpatialSteps - 1) ? 0 : temperatureField[i + 1, t - 1];
                        temperatureField[i, t] = temperatureField[i, t - 1] - alpha * (nextTemp - 2 * temperatureField[i, t - 1] + prevTemp);
                    }
                    currentTimeStep += StepSize;
                    await Task.Delay(10); // Добавляем небольшую задержку для обновления интерфейса
                    Application.Current.Dispatcher.Invoke(() => UpdateTemperatureField());
                }
            });
        }

        private void UpdateTemperatureField()
        {
            // Ваш код для обновления отображения температурного поля
            // Можно использовать это, чтобы обновлять график или выводить температуры в таблицу
            // Например:
            // TemperatureDisplay.Text = temperatureField[25, currentTimeStep].ToString(); // Для отображения температуры в середине шара
        }

        protected virtual void OnPropertyChanged([CallerMemberName] string propertyName = null)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }
    }
}