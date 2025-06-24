# Route-Rover-Mapping-shortest-path
The purpose of Route Rover, a clever travel planner, is to assist users in locating the three quickest and most secure road routes between cities in Uttrakhand
In the background, Route Rover uses a straightforward user interface, an effective shortest-path algorithm, and actual road network data.
# How It Works
# # Data Preprocessing (Python)
We use osmnx to fetch real road network data from OpenStreetMap for Uttarakhand. From this, we pick about 25 major cities and calculate the actual road distances  between them. To make things easy to read, we use geopy to turn coordinates into proper city names. All this clean data is saved in a CSV file, which the C++ program uses to build the route map
# # Shortest Path Calculation (C++)
Our C++ program reads real road distances for Uttarakhand from a CSV file. It then uses Dijkstra’s Algorithm along with Yen’s method to figure out the top 3      shortest and safest routes between the cities you choose.It takes your selected cities from a simple text file, calculates the best routes quickly, and saves the results — complete with city names and distances — in an output file you can easily read.
# # User Interface (Python)
  A simple Tkinter GUI makes it easy for you to pick a source city and a destination city within Uttarakhand. Once you click to find routes, the program shows the shortest route and 2 alternative safe routes — all displayed clearly with NetworkX and matplotlib so you can easily understand the path.
# How Python and C++ Talk
Python and C++ exchange information through simple text files (input.txt and output.txt). This keeps the system straightforward and avoids unnecessary complexity.
# Libraries & Tools Used
# # Language	         Libraries & Tools
- Python	            osmnx, geopy, Tkinter, matplotlib, NetworkX, pandas
- C++	                STL (unordered_map, priority_queue, fstream)
- Platforms	           Jupyter Notebook (Python), VS Code (C++)
# Future Scope
For now, Route Rover works for Uttarakhand, but it’s designed to be easily scaled to other Indian states. In the future, it can also include live traffic updates, safety scores, or real-time route suggestions.
# How to Run
- Preprocess Data — Run the Python script to generate the CSV for Uttarakhand.
- Launch GUI — Open the Python GUI script.
- Choose Cities — Select your source and destination cities.
- Find Routes — Click to get routes: the C++ program runs Dijkstra’s Algorithm and the Python GUI shows the results with clear visuals.
