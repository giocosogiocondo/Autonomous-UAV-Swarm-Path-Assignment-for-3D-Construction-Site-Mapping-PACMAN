PACMAN: Partitioned Autonomous Clustering for Multi‑drone Aerial Navigation
PACMAN is an AI‑enabled planning pipeline that converts raw GPS capture points into balanced, time‑efficient waypoint files for multi‑drone 3‑D mapping missions. It jointly handles coverage partitioning and tour optimization, enabling true inter‑drone cooperation and shortening total survey time.

✨ Key Features
Capacity‑Aware Restrained K‑Means: partitions mapping targets so each drone receives a near‑equal workload.
Pluggable TSP Solvers: choose between greedy, cheapest‑insertion, genetic algorithm, or ant‑colony methods to order waypoints.
End‑to‑End MATLAB Implementation: written and tested on MATLAB R2024b (v 24.2).
Reproducible Demo: includes sample datasets and scripts for immediate experimentation.


🛠 Requirements
MATLAB R2024b (24.2)
Optimization Toolbox (optional – enables the GA solver)

📑 Citing
If you use PACMAN in your research, please cite our accompanying paper:
@inproceedings{Myung 2025 PACMAN,
  title     = {Autonomous UAV Swarm Path Assignment for 3D Construction Site Mapping: PACMAN},
  author    = {Jaehyun Myung},
  booktitle = {International Conference on Construction Engineering and Project Management},
  year      = {2025}
}

🤝 Contributing
Pull requests are welcome! For major changes, please open an issue first to discuss what you would like to change.
Fork the repository.
Create your feature branch (git checkout -b feature/AmazingFeature).
Commit your changes (git commit -m 'Add some AmazingFeature').
Push to the branch (git push origin feature/AmazingFeature).
Open a Pull Request.

📜 License
This project is licensed under the MIT License – see the LICENSE file for details.

🙏 Acknowledgements
MATLAB® and the MATLAB logo are registered trademarks of MathWorks, Inc.



Last updated: 29 July 2025