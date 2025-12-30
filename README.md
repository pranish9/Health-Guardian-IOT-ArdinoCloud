<img width="975" height="650" alt="image" src="https://github.com/user-attachments/assets/1002a892-9f80-4117-be86-fd3dbd12dd4f" />

The aging global population faces numerous health challenges that require continuous and attentive monitoring. Traditional healthcare approaches rely heavily on periodic check-ups and hospital visits, which can be particularly burdensome for elderly individuals with mobility limitations (Raghunath Anekar et al., 1261). Furthermore, many health emergencies among the elderly occur suddenly between medical appointments, leading to delayed interventions and potentially worse outcomes.
Senior citizens often experience multiple health issues simultaneously, including cardiovascular disease, diabetes, respiratory problems, and other age-related conditions that require regular monitoring (Raghunath Anekar et al., 1261). Additionally, cognitive impairments such as Alzheimer's disease increase the risk of wandering and getting lost, creating safety concerns for both the elderly individuals and their caregivers (Poornima Ediga et al., 2024).
The problem is further compounded by the shortage of healthcare workers and the high costs associated with continuous professional care. Family members who serve as caregivers often cannot provide round-the-clock monitoring due to other responsibilities, creating gaps in care coverage that could lead to missed medical emergencies.
This project addresses these challenges by developing an IoT-based health monitoring system that provides continuous, real-time monitoring of vital health parameters, enables location tracking, and implements automated alerts for abnormal conditions. By doing so, it aims to enhance the quality of care for elderly individuals while reducing the burden on healthcare systems and family caregivers.


The Arduino Cloud dashboard for this system is configured with the following widgets:
1.	Heart Rate Line Graph
•	Widget Type: Advanced Chart
•	Variables: HeartRate
•	Chart Type: Line Graph
•	Y-axis range: 40-160 BPM
•	Threshold lines: 60 BPM (lower), 100 BPM (upper)
2.	SpO2 Percentage Indicator
•	Widget type: Guage
•	Variable: spo2
•	Range: 65-100%
3.	Temperature Gauge
•	Widget type: Thermometer
•	Variable: Temperature
•	Range: 
4.	GPS Location Map
•	Widget type: Map
•	Variable: Location
•	View: Satellite
•	Zoom Level: Street level

