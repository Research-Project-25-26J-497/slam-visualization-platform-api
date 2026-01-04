Here is the **README.md** file for your **Backend**. You can create a file named `README.md` in your `Backend` folder and paste this content.

```markdown
# M.A.N.T.I.S Backend - Quick Start Guide 🚀

This is the central control API for the M.A.N.T.I.S platform. It handles communication between the Robot, the Database (Neon), and the Frontend Visualization.

---

## 🛠️ Prerequisites

* **Python 3.9+** installed.
* **PostgreSQL Database URL** (from Neon Console) added to your `.env` file.

---

## ⚡ 1. Setup & Installation

**Open a terminal in the `Backend` folder:**

### Step A: Create Virtual Environment (First time only)
```powershell
# Create the environment
python -m venv venv

```

### Step B: Activate Virtual Environment (Every time)

You must do this *every time* you open a new terminal.

```powershell
# Windows PowerShell
.\venv\Scripts\activate

```

*(You should see `(venv)` appear at the start of your command line)*

### Step C: Install Dependencies

```powershell
pip install fastapi uvicorn sqlalchemy psycopg2-binary python-dotenv requests

```

---

## 🏗️ 2. Database Initialization

Run this script once to create the necessary tables (`annotations`, `map_points`) in your Neon database. **Warning:** This script may clear existing map data if configured to reset.

```powershell
python create_db.py

```

*Expected Output:* `✅ DATABASE CONNECTION SUCCESSFUL!`

---

## 🟢 3. Start the Server

This launches the API Brain. Keep this terminal open!

```powershell
python -m uvicorn main:app --reload

```

* **API URL:** `http://localhost:8000`
* **Swagger Docs:** `http://localhost:8000/docs` (Test endpoints here)

---

## 🤖 4. Run the Robot Simulator (Optional)

If you don't have a real robot connected, you can run the "Virtual Robot" to generate map data.

**Option A: Manual Start**
Open a **new terminal**, activate venv, and run:

```powershell
python simulate_slam.py

```

**Option B: Via Frontend**
Click the **"▶ START SIMULATION"** button on the Frontend Sidebar.

---

## 🆘 Troubleshooting

* **"Module not found" error?**
* Make sure you activated the venv: `.\venv\Scripts\activate`


* **Database connection failed?**
* Check your `.env` file. It should look like:
`DATABASE_URL="postgresql://user:pass@ep-xyz.aws.neon.tech/neondb?sslmode=require"`


* **Port 8000 already in use?**
* Kill the old process or restart your computer.



---

## 📂 Project Structure

* `main.py` -> The API Server (Endpoints & WebSocket).
* `simulate_slam.py` -> The Virtual Robot script.
* `database/`
* `models.py` -> Database table definitions.
* `db.py` -> Connection settings.



```

### ✅ How to use this for your presentation
You can print this out or keep it open on a side screen so you never forget the exact commands to type during your live demo!

```