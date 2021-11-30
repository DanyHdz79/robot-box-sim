import flask
from flask.json import jsonify
import uuid
from actRobots import Robot, Box, Maze

actInt = {}
app = flask.Flask(__name__)

@app.route("/actInt", methods=["POST"])
def create():
    global actInt
    id = str(uuid.uuid4())
    actInt[id] = Maze()
    return "ok", 201, {'Location': f"/actInt/{id}"}

@app.route("/actInt/<id>", methods=["GET"])
def queryState(id):
    global model
    model = actInt[id]
    model.step()
    agents = model.schedule.agents

    listRobots = []
    listBoxes = []
    listStacks = model.list_stacks

    for agent in agents:
        if(isinstance(agent, Robot)):
            listRobots.append({"id": agent.unique_id, "x": agent.pos[0], "y": agent.pos[1], "carry": agent.carry})
        elif(isinstance(agent, Box)):
            listBoxes.append({"id": agent.unique_id, "x": agent.pos[0], "y": agent.pos[1], "active": agent.active})

    return jsonify({"robots": listRobots, "boxes": listBoxes, "stacks": listStacks})

app.run()