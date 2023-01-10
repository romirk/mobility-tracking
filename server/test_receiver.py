import pytest

from server.receiver import Server


@pytest.fixture
def app():
    app, _ = Server.create_server()
    app.config.update({
        "TESTING": True,
    })

    # other setup can go here

    yield app

    # clean up / reset resources here


@pytest.fixture()
def client(app):
    return app.test_client()


@pytest.fixture()
def runner(app):
    return app.test_cli_runner()


def test_sensor(client):
    response = client.post("/sensors", json={"CO2": 42})
    assert response.status_code == 200
