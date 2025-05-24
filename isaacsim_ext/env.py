from functools import lru_cache

from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    save_frame_interval: int = 10
    conveni_product_stack_rows: int = 3
    conveni_product_stack_columns: int = 2
    conveni_product_stack_gap: int = 2

    model_config = SettingsConfigDict(env_file=".env", frozen=True)


@lru_cache
def get_settings() -> Settings:
    return Settings()
