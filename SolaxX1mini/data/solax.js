var ChartInizialized = false;
var myChart;
var LastxValueLen = 0;
var xValues = [];
var Power = [];
var Energy = [];


function AutoReload() {
  UpdateJson();
  UpdateChart();
  setTimeout('AutoReload()', 1000);
}

function UpdateJson()
{
  nocache = "&nocache="+Math.random() * 1000000;
  var request = new XMLHttpRequest();
  request.onload = function() {
    if (this.responseText != null) {
      var jsonData = JSON.parse(this.responseText);
      var runtime_days = Math.floor(jsonData.runtime / 86400);
      var runtime_hours = Math.floor((jsonData.runtime % 86400) / 3600);
      var runtime_minutes = Math.floor(((jsonData.runtime % 86400) % 3600) / 60);
      var runtime_seconds = jsonData.runtime % 60;
      var runtime_total_days = Math.floor(jsonData.runtime_total / 24);
      var runtime_total_hours = Math.floor(jsonData.runtime_total % 24);
      var dc_power = Math.round(jsonData.dc_voltage * jsonData.dc_current);
      var power_loss_watt = Math.round((jsonData.dc_voltage * jsonData.dc_current) - jsonData.ac_power);
      var power_loss_percent = Math.round(((jsonData.dc_voltage * jsonData.dc_current) - jsonData.ac_power ) / (jsonData.dc_voltage * jsonData.dc_current)*100);
      var runtime;
      
      if(runtime_days > 0) {
        runtime = String(runtime_days) + 'd ' + String(runtime_hours) + 'h ' + String(runtime_minutes) + 'm ' + String(runtime_seconds) + 's';
      }
      else if(runtime_hours > 0) {
        runtime = String(runtime_hours) + 'h ' + String(runtime_minutes) + 'm ' + String(runtime_seconds) + 's';
      }
      else if(runtime_minutes > 0) {
        runtime = String(runtime_minutes) + 'm ' + String(runtime_seconds) + 's';
      }
      else {
        runtime = String(runtime_seconds) + 's';
      }
        
      if(isNaN(power_loss_percent)) power_loss_percent = 0;
      
      document.getElementById("mode_name").innerHTML = String(jsonData.mode_name);
      document.getElementById("temperature").innerHTML = String(jsonData.temperature) + ' &deg;C';
      document.getElementById("error_bits").innerHTML = String(jsonData.error_bits);
      document.getElementById("dc_voltage").innerHTML = String(jsonData.dc_voltage) + ' V';
      document.getElementById("dc_current").innerHTML = String(jsonData.dc_current) + ' A';
      document.getElementById("dc_power").innerHTML = String(dc_power) + ' W';
      document.getElementById("ac_voltage").innerHTML = String(jsonData.ac_voltage) + ' V';
      document.getElementById("ac_current").innerHTML = String(jsonData.ac_current) + ' A';
      document.getElementById("ac_frequency").innerHTML = String(jsonData.ac_frequency) + ' Hz';
      document.getElementById("ac_power").innerHTML = String(jsonData.ac_power) + ' W (' + String(jsonData.avg_ac_power) + ' W / ' + String(jsonData.max_ac_power) + ' W)';
      document.getElementById("power_loss").innerHTML = String(power_loss_watt) + ' W (' + String(power_loss_percent) + ' %)';
      document.getElementById("energy_today").innerHTML = String(jsonData.energy_today + ' kWh');
      document.getElementById("energy_total").innerHTML = String(jsonData.energy_total) + ' kWh';
      document.getElementById("runtime").innerHTML = runtime;
      document.getElementById("runtime_total").innerHTML = String(runtime_total_days) + 'd ' + String(runtime_total_hours) + 'h ';
      document.getElementById("wifi_ssid").innerHTML = String(jsonData.wifi_ssid);
      document.getElementById("wifi_ip").innerHTML = String(jsonData.wifi_ip);
      document.getElementById("wifi_rssi").innerHTML = String(jsonData.wifi_rssi) + ' dBm';

      Power = jsonData.stat_ac_power;
      Energy = jsonData.stat_energy_today;
      xValues = []
      for (var i = 0; i < Energy.length; i++) {
        Energy[i] /= 10.0;
        xValues.push(i);
      }
    }
  }
  request.open("GET", "update.json" + nocache, true);
  request.send(null);
}

function UpdateChart() {
  if(ChartInizialized) {
    if (LastxValueLen < xValues.length) {
      LastxValueLen = xValues.length;
      myChart.data.labels = xValues;
      myChart.data.datasets[0].data = Power;
      myChart.data.datasets[1].data = Energy;
      myChart.update();
    }
  }
  else {
      const ctx = document.getElementById("chart")

      myChart = new Chart(ctx, {
        type: "line",
        data: {
          labels: xValues,
          datasets: [{ 
            label: 'Leistung [W]',
            data: Power,
            yAxisID: 'y',
            borderColor: "rgba(42,98,193,1)",
            backgroundColor: "rgba(42,98,193,0.1)",
            fill: true
          }, { 
            label: 'Energie [kW/h]',
            data: Energy,
            yAxisID: 'y1',
            borderColor: "rgba(98,98,98,1)",
            fill: false,
            stepped: true
          }]
        },
        options: {
          responsive: true,
          interaction: {
            mode: 'index',
            intersect: false,
           },
          stacked: false,
          scales: {
            x: {
              display: false
            },
            y: {
              type: 'linear',
              title: {
                display: false,
                text: 'Leistung [W]'
              },
              min: 0,
              max: 700,
              display: true,
              position: 'left',
            },
            y1: {
              type: 'linear',
              title: {
                display: false,
                text: 'Energie [kW/h]'
              },
              min: 0,
              max: 3.5,
              display: true,
              position: 'right',
              grid: {
                drawOnChartArea: false, 
              },
            },
          },
          legend: {display: false},
          animations: {
            tension: {
              duration: 1500,
              easing: 'linear',
              from: 0,
              to: 0.5,
              loop: false
            }
          }
        }
      });
    ChartInizialized = true;
  }
}
