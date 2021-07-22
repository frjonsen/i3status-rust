use std::collections::HashMap;
use std::env;
use std::time::Duration;

use crossbeam_channel::Sender;
use serde_derive::Deserialize;

use crate::blocks::{Block, ConfigBlock, Update};
use crate::config::SharedConfig;
use crate::de::deserialize_duration;
use crate::errors::*;
use crate::formatting::value::Value;
use crate::formatting::FormatTemplate;
use crate::http::{self, HttpResponse};
use crate::protocol::i3bar_event::{I3BarEvent, MouseButton};
use crate::scheduler::Task;
use crate::widgets::{text::TextWidget, I3BarWidget, State};

const OPENWEATHERMAP_API_KEY_ENV: &str = "OPENWEATHERMAP_API_KEY";
const OPENWEATHERMAP_CITY_ID_ENV: &str = "OPENWEATHERMAP_CITY_ID";
const OPENWEATHERMAP_PLACE_ENV: &str = "OPENWEATHERMAP_PLACE";

#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "name", rename_all = "lowercase")]
pub enum WeatherService {
    OpenWeatherMap {
        #[serde(default = "WeatherService::getenv_openweathermap_api_key")]
        api_key: Option<String>,
        #[serde(default = "WeatherService::getenv_openweathermap_city_id")]
        city_id: Option<String>,
        #[serde(default = "WeatherService::getenv_openweathermap_place")]
        place: Option<String>,
        coordinates: Option<(String, String)>,
        units: OpenWeatherMapUnits,
        #[serde(default = "WeatherService::default_lang")]
        lang: Option<String>,
        #[serde(default = "WeatherService::default_api_url")]
        api_url: String,
    },
}

impl WeatherService {
    fn getenv_openweathermap_api_key() -> Option<String> {
        env::var(OPENWEATHERMAP_API_KEY_ENV).ok()
    }
    fn getenv_openweathermap_city_id() -> Option<String> {
        env::var(OPENWEATHERMAP_CITY_ID_ENV).ok()
    }
    fn getenv_openweathermap_place() -> Option<String> {
        env::var(OPENWEATHERMAP_PLACE_ENV).ok()
    }
    fn default_lang() -> Option<String> {
        Some("en".to_string())
    }
    fn default_api_url() -> String {
        "https://api.openweathermap.org/data/2.5".to_string()
    }
}

#[derive(Copy, Clone, Debug, Deserialize, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum OpenWeatherMapUnits {
    Metric,
    Imperial,
}

#[derive(Clone)]
pub struct Location {
    latitude: f64,
    longitude: f64,
    name: String,
}

pub struct Weather {
    id: usize,
    weather: TextWidget,
    format: FormatTemplate,
    weather_keys: HashMap<&'static str, Value>,
    service: WeatherService,
    update_interval: Duration,
    autolocate: bool,
    // Cached location entry, will be filled later
    location: Option<Location>,
}

fn malformed_json_error() -> Error {
    BlockError("weather".to_string(), "Malformed JSON.".to_string())
}

// TODO: might be good to allow for different geolocation services to be used, similar to how we have `service` for the weather API
fn find_ip_location() -> Result<Option<String>> {
    let http_call_result = http::http_get_json(
        "https://ipapi.co/json/",
        Some(Duration::from_secs(3)),
        vec![],
    )?;

    let city = http_call_result
        .content
        .pointer("/city")
        .map(|v| v.as_str())
        .flatten()
        .map(|s| s.to_string());

    Ok(city)
}

// Compute the Australian Apparent Temperature (AT),
// using the metric formula found on Wikipedia.
// If using imperial units, we must first convert to metric.
fn australian_apparent_temp(
    raw_temp: f64,
    raw_humidity: f64,
    raw_wind_speed: f64,
    units: OpenWeatherMapUnits,
) -> f64 {
    let metric = units == OpenWeatherMapUnits::Metric;

    let temp_celsius = if units == OpenWeatherMapUnits::Metric {
        raw_temp
    } else {
        // convert Fahrenheit to Celsius
        (raw_temp - 32.0) * 0.556
    };

    let exponent = 17.27 * temp_celsius / (237.7 + temp_celsius);
    let water_vapor_pressure = raw_humidity * 0.06105 * exponent.exp();

    let metric_wind_speed = if metric {
        raw_wind_speed
    } else {
        // convert mph to m/s
        raw_wind_speed * 0.447
    };

    let metric_apparent_temp =
        temp_celsius + 0.33 * water_vapor_pressure - 0.7 * metric_wind_speed - 4.0;

    if metric {
        metric_apparent_temp
    } else {
        1.8 * metric_apparent_temp + 32.0
    }
}

// Convert wind direction in azimuth degrees to abbreviation names
fn convert_wind_direction(direction_opt: Option<f64>) -> String {
    match direction_opt {
        Some(direction) => match direction.round() as i64 {
            24..=68 => "NE".to_string(),
            69..=113 => "E".to_string(),
            114..=158 => "SE".to_string(),
            159..=203 => "S".to_string(),
            204..=248 => "SW".to_string(),
            249..=293 => "W".to_string(),
            294..=338 => "NW".to_string(),
            _ => "N".to_string(),
        },
        None => "-".to_string(),
    }
}

fn configuration_error(msg: &str) -> crate::Error {
    ConfigurationError("weather".to_owned(), msg.to_owned())
}

impl Weather {
    fn find_location(&self) -> Result<Location> {
        match &self.service {
            WeatherService::OpenWeatherMap {
                api_key: api_key_opt,
                city_id,
                place,
                coordinates,
                units: _,
                lang: _,
                api_url,
            } => {
                if api_key_opt.is_none() {
                    return Err(configuration_error(&format!(
                        "Missing member 'service.api_key'. Add the member or configure with the environment variable {}",
                        OPENWEATHERMAP_API_KEY_ENV.to_string())));
                }

                let geoip_city = if self.autolocate {
                    find_ip_location().ok().unwrap_or(None) // If geo location fails, try other configuration methods
                } else {
                    None
                };

                let location_query = if let Some(city) = geoip_city {
                    format!("q={}", city)
                } else if let Some(cid) = city_id.as_ref() {
                    format!("id={}", cid)
                } else if let Some(p) = place.as_ref() {
                    format!("q={}", p)
                } else if let Some((lat, lon)) = coordinates {
                    // Even if we already have the coordinates we must still
                    // make the API call to get the name of the location
                    format!("lat={}&lon={}", lat, lon)
                } else if self.autolocate {
                    return Err(configuration_error(
                        "weather is configured to use geolocation, but it could not be obtained",
                    ));
                } else {
                    return Err(configuration_error(&format!(
                        "Either 'service.city_id' or 'service.place' must be provided. Add one to your config file or set with the environment variables {} or {}",
                        OPENWEATHERMAP_CITY_ID_ENV.to_string(),
                        OPENWEATHERMAP_PLACE_ENV.to_string())));
                };

                let api_key = api_key_opt.as_ref().unwrap();

                // This uses the "Current Weather Data" API endpoint
                // Refer to https://openweathermap.org/current
                let openweather_url = &format!(
                    "{api_url}/weather?{location_query}&appid={api_key}",
                    api_url = api_url,
                    location_query = location_query,
                    api_key = api_key,
                );

                let output =
                    http::http_get_json(openweather_url, Some(Duration::from_secs(3)), vec![])?;
                let json = Weather::get_curl_output(output)?;

                // Try to convert an API error into a block error.
                if let Some(val) = json.get("message") {
                    return Err(BlockError(
                        "weather".to_string(),
                        format!("API Error: {}", val.as_str().unwrap()),
                    ));
                };

                let raw_lat = json
                    .pointer("/coord/lat")
                    .and_then(serde_json::Value::as_f64)
                    .ok_or_else(malformed_json_error)?;

                let raw_lon = json
                    .pointer("/coord/lon")
                    .and_then(serde_json::Value::as_f64)
                    .ok_or_else(malformed_json_error)?;

                let raw_location_name = json
                    .pointer("/name")
                    .and_then(serde_json::Value::as_str)
                    .map(str::to_string)
                    .ok_or_else(malformed_json_error)?;

                let coordinates = Location {
                    latitude: raw_lat,
                    longitude: raw_lon,
                    name: raw_location_name,
                };
                Ok(coordinates)
            }
        }
    }

    fn get_curl_output(response: HttpResponse<serde_json::Value>) -> Result<serde_json::Value> {
        // All 300-399 and >500 http codes should be considered as temporary error,
        // and not result in block error, i.e. leave the output empty.
        if (response.code >= 300 && response.code < 400) || response.code >= 500 {
            return Err(BlockError(
                "weather".to_owned(),
                format!("Invalid result from curl: {}", response.code),
            ));
        };

        Ok(response.content)
    }

    fn update_weather(&mut self) -> Result<()> {
        match &self.service {
            WeatherService::OpenWeatherMap {
                api_key: api_key_opt,
                city_id: _,
                place: _,
                units,
                coordinates: _,
                lang,
                api_url,
            } => {
                if api_key_opt.is_none() {
                    return Err(configuration_error(&format!(
                        "Missing member 'service.api_key'. Add the member or configure with the environment variable {}",
                        OPENWEATHERMAP_API_KEY_ENV.to_string())));
                }

                if self.location.is_none() {
                    self.location = Some(self.find_location()?);
                }

                let location = self
                    .location
                    .as_ref()
                    .expect("Location was not cached after successful location lookup");

                let api_key = api_key_opt.as_ref().unwrap();

                let location_query =
                    format!("lat={:.4}&lon={:.4}", location.latitude, location.longitude);

                // This uses the "Current Weather Data" API endpoint
                // Refer to https://openweathermap.org/current
                let openweather_url = &format!(
                    "{api_url}/weather?{location_query}&appid={api_key}&units={units}&lang={lang}",
                    api_url = api_url,
                    location_query = location_query,
                    api_key = api_key,
                    units = match *units {
                        OpenWeatherMapUnits::Metric => "metric",
                        OpenWeatherMapUnits::Imperial => "imperial",
                    },
                    lang = lang.as_ref().unwrap(),
                );

                let output =
                    http::http_get_json(openweather_url, Some(Duration::from_secs(3)), vec![])?;

                let json = Weather::get_curl_output(output)?;

                // Try to convert an API error into a block error.
                if let Some(val) = json.get("message") {
                    return Err(BlockError(
                        "weather".to_string(),
                        format!("API Error: {}", val.as_str().unwrap()),
                    ));
                };

                let raw_weather = json
                    .pointer("/weather/0/main")
                    .and_then(|v| v.as_str())
                    .ok_or_else(malformed_json_error)?
                    .to_string();

                let raw_weather_verbose = json
                    .pointer("/weather/0/description")
                    .and_then(|v| v.as_str())
                    .ok_or_else(malformed_json_error)?
                    .to_string();

                let raw_temp = json
                    .pointer("/main/temp")
                    .and_then(|v| v.as_f64())
                    .ok_or_else(malformed_json_error)?;

                let raw_humidity = json
                    .pointer("/main/humidity")
                    .map_or(Some(0.0), |v| v.as_f64()) // provide default value 0.0
                    .ok_or_else(malformed_json_error)?;

                let raw_wind_speed: f64 = json
                    .pointer("/wind/speed")
                    .map_or(Some(0.0), |v| v.as_f64()) // provide default value 0.0
                    .ok_or_else(malformed_json_error)?; // error when conversion to f64 fails

                let raw_wind_direction: Option<f64> = json
                    .pointer("/wind/deg")
                    .map_or(Some(None), |v| v.as_f64().map(Some)) // provide default value None
                    .ok_or_else(malformed_json_error)?; // error when conversion to f64 fails

                let raw_location = json
                    .pointer("/name")
                    .and_then(|v| v.as_str())
                    .map(|s| s.to_string())
                    .ok_or_else(malformed_json_error)?;

                self.weather.set_icon(match raw_weather.as_str() {
                    "Clear" => "weather_sun",
                    "Rain" | "Drizzle" => "weather_rain",
                    "Clouds" | "Fog" | "Mist" => "weather_clouds",
                    "Thunderstorm" => "weather_thunder",
                    "Snow" => "weather_snow",
                    _ => "weather_default",
                })?;

                let kmh_wind_speed = if *units == OpenWeatherMapUnits::Metric {
                    raw_wind_speed * 3600.0 / 1000.0
                } else {
                    // convert mph to m/s, then km/h
                    (raw_wind_speed * 0.447) * 3600.0 / 1000.0
                };

                let apparent_temp =
                    australian_apparent_temp(raw_temp, raw_humidity, raw_wind_speed, *units);

                self.weather_keys = map!(
                    "weather" => Value::from_string(raw_weather),
                    "weather_verbose" => Value::from_string(raw_weather_verbose),
                    "temp" => Value::from_integer(raw_temp as i64).degrees(),
                    "humidity" => Value::from_integer(raw_humidity as i64),
                    "apparent" => Value::from_integer(apparent_temp as i64).degrees(),
                    "wind" => Value::from_float(raw_wind_speed),
                    "wind_kmh" => Value::from_float(kmh_wind_speed),
                    "direction" => Value::from_string(convert_wind_direction(raw_wind_direction)),
                    "location" => Value::from_string(raw_location),
                );
                Ok(())
            }
        }
    }
}

#[derive(Deserialize, Debug, Clone, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum ForecastWhen {
    Today,
    Tomorrow,
}

impl Default for ForecastWhen {
    fn default() -> Self {
        ForecastWhen::Today
    }
}

#[derive(Deserialize, Debug, Clone, PartialEq, Default)]
pub struct ForecastConfig {
    #[serde(default)]
    when: ForecastWhen,
    #[serde(default = "ForecastConfig::default_time_of_day")]
    // API only returns forecasts at full hours, i.e 13:00, 14:00, etc
    at: u8,
}

impl ForecastConfig {
    fn default_time_of_day() -> u8 {
        0
    }
}

#[derive(Deserialize, Debug, Clone)]
#[serde(deny_unknown_fields)]
pub struct WeatherConfig {
    #[serde(
        default = "WeatherConfig::default_interval",
        deserialize_with = "deserialize_duration"
    )]
    pub interval: Duration,
    #[serde(default)]
    pub format: FormatTemplate,
    pub service: WeatherService,
    #[serde(default)]
    pub autolocate: bool,
    #[serde(default)]
    pub forecast: ForecastConfig,
}

impl WeatherConfig {
    fn default_interval() -> Duration {
        Duration::from_secs(600)
    }
}

impl ConfigBlock for Weather {
    type Config = WeatherConfig;

    fn new(
        id: usize,
        block_config: Self::Config,
        shared_config: SharedConfig,
        _tx_update_request: Sender<Task>,
    ) -> Result<Self> {
        // No need to check for negatives since it's unsigned
        if block_config.forecast.at > 23 {
            return Err(configuration_error("'at' is not a valid hour"));
        }

        Ok(Weather {
            id,
            weather: TextWidget::new(id, 0, shared_config),
            format: block_config
                .format
                .with_default("{weather} {temp}\u{00b0}")?,
            weather_keys: HashMap::new(),
            service: block_config.service,
            update_interval: block_config.interval,
            autolocate: block_config.autolocate,
            location: None,
        })
    }
}

impl Block for Weather {
    fn update(&mut self) -> Result<Option<Update>> {
        match self.update_weather() {
            Ok(_) => {
                self.weather
                    .set_texts(self.format.render(&self.weather_keys)?);
                self.weather.set_state(State::Idle)
            }
            Err(BlockError(block, _)) | Err(InternalError(block, _, _)) if block == "curl" => {
                // Ignore curl/api errors
                self.weather.set_icon("weather_default")?;
                self.weather.set_text("×".to_string());
                self.weather.set_state(State::Warning)
            }
            Err(err) => {
                self.weather.set_text(format!("weather error {}:", err));
                self.weather.set_state(State::Critical);
            }
        }

        Ok(Some(self.update_interval.into()))
    }

    fn view(&self) -> Vec<&dyn I3BarWidget> {
        vec![&self.weather]
    }

    fn click(&mut self, event: &I3BarEvent) -> Result<()> {
        if let MouseButton::Left = event.button {
            self.update()?;
        }
        Ok(())
    }

    fn id(&self) -> usize {
        self.id
    }
}

#[cfg(test)]
mod tests {
    use super::{ForecastConfig, Weather, WeatherConfig};
    use crate::config::SharedConfig;
    use crate::widgets::text::TextWidget;
    use httpmock::MockServer;
    use std::collections::HashMap;
    use test_case::test_case;

    fn get_weather_block(api_url: String, config: &[&str]) -> Weather {
        let mut options = config.join(", ");

        if !config.is_empty() {
            options = format!(", {}", options);
        }

        let block_config = toml::from_str::<WeatherConfig>(
            &format!(r#"
                "service" = {{"name" = "openweathermap", "units" = "metric", "api_key" = "mock_api_key", "api_url" = "{}"{}}}
            "#, api_url, options),
        )
        .unwrap();
        Weather {
            id: 0,
            weather: TextWidget::new(0, 0, SharedConfig::default()),
            weather_keys: HashMap::new(),
            service: block_config.service,
            update_interval: block_config.interval,
            autolocate: block_config.autolocate,
            location: None,
            format: block_config
                .format
                .with_default("{weather} {temp}\u{00b0}")
                .unwrap(),
        }
    }

    #[test_case("\"when\" = \"today\"\n\"at\" = 0")]
    #[test_case(r#""when" = "today""#)]
    #[test_case("\"when\" = \"tomorrow\"\n\"at\" = 16")]
    #[test_case("\n")] // Can't be empty
    fn test_forecast_valid_config(config: &str) {
        toml::from_str::<ForecastConfig>(config).expect("Valid configuration failed");
    }

    #[test_case(r#""when" = "yesterday""#)]
    fn test_forecast_invalid_config(config: &str) {
        toml::from_str::<ForecastConfig>(config).expect_err("Invalid configuration was allowed");
    }

    #[test]
    fn test_forecast_default() {
        let specified =
            toml::from_str::<ForecastConfig>("\"when\" = \"today\"\n\"at\" = 0").unwrap();
        let defaults = toml::from_str::<ForecastConfig>("").unwrap();
        assert_eq!(specified, defaults)
    }

    #[test_case(&[r#""city_id" = "12345""#])]
    #[test_case(&[r#""place" = "London""#])]
    #[test_case(&[r#""coordinates" = ["18.4856", "4.3256"]"#])]
    fn test_get_location_by_query(config: &[&str]) {
        let server = MockServer::start();

        let weather_mock = server.mock(|when, then| {
            when.path_contains("/weather");
            then.status(200)
                // A real response contains a lot more, but this is the only part
                // relevant to the test
                .body(r#"{ "coord": { "lat": 18.4856, "lon": 4.3256 }, "name": "London" }"#);
        });

        let weather = get_weather_block(format!("http://127.0.0.1:{}", server.port()), config);
        let location = weather.find_location().unwrap();

        weather_mock.assert();
        assert!((location.latitude - 18.4856).abs() < f64::EPSILON);
        assert!((location.longitude - 4.3256).abs() < f64::EPSILON);
        assert_eq!(location.name, "London");
    }

    #[test]
    }
}
