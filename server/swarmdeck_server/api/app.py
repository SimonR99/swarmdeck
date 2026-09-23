"""FastAPI composition root: lifecycle, collaborator wiring and registration."""

from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager

from fastapi import FastAPI

from . import state
from .adapter_socket import router as adapter_router
from .autonomy_routes import router as autonomy_router
from .deployment_raster import deployment_raster_loop
from .gui_socket import router as gui_router
from .http_routes import router as http_router
from .map_routes import CachedEpochStore, command_guard
from .replica_views import router as replica_views_router


@asynccontextmanager
async def lifespan(_: FastAPI):
    if not state.CONFIG:
        state.load_config()
    state.settings_store.load()
    state.apply_review_radii(state.settings_store.value)
    state.load_review()
    tasks = [
        asyncio.create_task(state.state_loop()),
        asyncio.create_task(state.network_loop()),
        asyncio.create_task(state.session_loop()),
        asyncio.create_task(deployment_raster_loop()),
    ]
    yield
    for task in tasks:
        task.cancel()


def create_app() -> FastAPI:
    epoch_store = CachedEpochStore()
    state.registry.epoch_store = lambda: epoch_store
    state.registry.command_guard = command_guard
    application = FastAPI(title="SwarmDeck", lifespan=lifespan)
    for router in (
        autonomy_router,
        replica_views_router,
        http_router,
        gui_router,
        adapter_router,
    ):
        application.include_router(router)
    return application


app = create_app()
