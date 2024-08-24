#include "LogLoader.hpp"
#include <iomanip>
#include <iostream>
#include <filesystem>
#include <future>
#include <regex>
#define CPPHTTPLIB_OPENSSL_SUPPORT
#include <httplib.h>

namespace fs = std::filesystem;

LogLoader::LogLoader(const LogLoader::Settings& settings)
	: _settings(settings)
{
	// Disable mavsdk noise
	// mavsdk::log::subscribe([](...) {
	//  // https://mavsdk.mavlink.io/main/en/cpp/guide/logging.html
	//  return true;
	// });

	std::cout << std::fixed << std::setprecision(8);

	// Ensure proper directory syntax
	if (_settings.logging_directory.back() != '/') {
		_settings.logging_directory += '/';
	}

	fs::create_directories(_settings.logging_directory);
}

void LogLoader::stop()
{
	{
		std::lock_guard<std::mutex> lock(_exit_cv_mutex);
		_should_exit = true;
	}
	_exit_cv.notify_one();
}

bool LogLoader::wait_for_mavsdk_connection(double timeout_ms)
{
	std::cout << "Connecting to " << _settings.mavsdk_connection_url << std::endl;
	_mavsdk = std::make_shared<mavsdk::Mavsdk>(mavsdk::Mavsdk::Configuration(1, MAV_COMP_ID_ONBOARD_COMPUTER,
			true)); // Emit heartbeats (Client)
	auto result = _mavsdk->add_any_connection(_settings.mavsdk_connection_url);

	if (result != mavsdk::ConnectionResult::Success) {
		std::cout << "Connection failed: " << result << std::endl;
		return false;
	}

	auto system = _mavsdk->first_autopilot(timeout_ms);

	if (!system) {
		std::cout << "Timed out waiting for system" << std::endl;
		return false;
	}

	std::cout << "Connected to autopilot" << std::endl;

	// MAVSDK plugins
	_log_files = std::make_shared<mavsdk::LogFiles>(system.value());
	_telemetry = std::make_shared<mavsdk::Telemetry>(system.value());

	return true;
}

void LogLoader::run()
{
	auto upload_thread = std::thread(&LogLoader::upload_logs_thread, this);

	while (!_should_exit) {
		// Check if vehicle is armed or if the logger is running
		// bool logger_running = _telemetry->sys_status_sensors().enabled & MAV_SYS_STATUS_LOGGING;
		bool logger_running = false;
		bool vehicle_armed = _telemetry->armed();

		if (logger_running || vehicle_armed) {
			_loop_disabled = true;
			std::this_thread::sleep_for(std::chrono::seconds(1));
			continue;

		} else if (_loop_disabled) {
			_loop_disabled = false;
			// Stall for a few seconds to allow logger to finish writing
			std::this_thread::sleep_for(std::chrono::seconds(3));
		}

		if (!request_log_entries()) {
			std::cout << "Failed to get logs" << std::endl;
			std::this_thread::sleep_for(std::chrono::seconds(5));
			continue;
		}

		std::cout << "Found " << _log_entries.size() << " logs" << std::endl;

		// Pretty print them
		int indexWidth = std::to_string(_log_entries.size() - 1).length();

		for (const auto& e : _log_entries) {
			std::cout << std::setw(indexWidth) << std::right << e.id << "\t"  // Right-align the index
				  << e.date << "\t" << std::fixed << std::setprecision(2) << e.size_bytes / 1e6 << "MB" << std::endl;
		}

		// If we have no logs, just download the latest
		auto most_recent_log = find_most_recent_log();

		if (most_recent_log.date.empty()) {
			download_first_log();

		} else {
			// Otherwise download all logs more recent than the latest log we have locally
			download_logs_greater_than(most_recent_log);
		}

		// Periodically request log list
		if (!_should_exit) {
			std::unique_lock<std::mutex> lock(_exit_cv_mutex);
			std::cout << "Sleeping..." << std::endl;
			_exit_cv.wait_for(lock, std::chrono::seconds(10), [this] { return _should_exit.load(); });
		}
	}

	upload_thread.join();
}

bool LogLoader::request_log_entries()
{
	std::cout << "Requesting log list..." << std::endl;
	auto entries_result = _log_files->get_entries();

	if (entries_result.first != mavsdk::LogFiles::Result::Success) {
		return false;
	}

	_log_entries = entries_result.second;

	return true;
}

void LogLoader::download_first_log()
{
	std::cout << "No local logs found, downloading latest" << std::endl;
	auto entry = _log_entries.back();
	download_log(entry);
}

void LogLoader::download_logs_greater_than(const mavsdk::LogFiles::Entry& most_recent)
{
	// Check which logs need to be downloaded
	for (auto& entry : _log_entries) {

		if (_should_exit) {
			return;
		}

		bool new_log = entry.id > most_recent.id;
		bool partial_log = (entry.id == most_recent.id) && (entry.size_bytes > most_recent.size_bytes);

		if (new_log || partial_log) {
			if (partial_log) {
				std::cout << "Incomplete log, re-downloading..." << std::endl;
				std::cout << "size actual/downloaded: " << entry.size_bytes << "/" << most_recent.size_bytes << std::endl;

				auto log_path = filepath_from_entry(entry);

				if (fs::exists(log_path)) {
					fs::remove(log_path);
				}
			}

			download_log(entry);
		}
	}
}

bool LogLoader::download_log(const mavsdk::LogFiles::Entry& entry)
{
	auto prom = std::promise<mavsdk::LogFiles::Result> {};
	auto future_result = prom.get_future();

	auto download_path = filepath_from_entry(entry);

	std::cout << "Downloading  " << download_path << std::endl;

	// Mark the file as currently being downloaded
	{
		std::lock_guard<std::mutex> lock(_current_download_mutex);
		_current_download.second = false;
		_current_download.first = std::filesystem::path(download_path).filename().string();
	}

	auto time_start = std::chrono::steady_clock::now();

	_log_files->download_log_file_async(
		entry,
		download_path,
	[&prom, &entry, &time_start, this](mavsdk::LogFiles::Result result, mavsdk::LogFiles::ProgressData progress) {

		if (_download_cancelled) return;

		auto now = std::chrono::steady_clock::now();

		// Calculate data rate in Kbps
		double rate_kbps = ((progress.progress * entry.size_bytes * 8.0)) / std::chrono::duration_cast<std::chrono::milliseconds>(now -
				   time_start).count(); // Convert bytes to bits and then to Kbps

		if (_should_exit) {
			_download_cancelled = true;
			prom.set_value(mavsdk::LogFiles::Result::Timeout);
			std::cout << std::endl << "Download cancelled.. exiting" << std::endl;
			return;
		}

		std::cout << "Downloading:  "
			  << std::setw(24) << std::left << entry.date
			  << std::setw(8) << std::fixed << std::setprecision(2) << entry.size_bytes / 1e6 << "MB"
			  << std::setw(6) << std::right << int(progress.progress * 100.0f) << "%"
			  << std::setw(12) << std::fixed << std::setprecision(2) << rate_kbps << " Kbps"
			  << std::flush << std::endl;

		if (result != mavsdk::LogFiles::Result::Next) {
			double seconds = std::chrono::duration_cast<std::chrono::milliseconds>(now - time_start).count() / 1000.;
			std::cout << "Finished in " << std::setprecision(2) << seconds << " seconds" << std::endl;
			prom.set_value(result);
		}
	});

	auto result = future_result.get();

	std::cout << std::endl;

	bool success = result == mavsdk::LogFiles::Result::Success;

	if (success) {
		std::lock_guard<std::mutex> lock(_current_download_mutex);
		_current_download.second = true;

	} else {
		std::cout << "Download failed" << std::endl;
	}

	return success;
}

void LogLoader::upload_logs_thread()
{
	if (!_settings.upload_enabled) {
		return;
	}

	// Short startup delay to allow the download thread to start re-downloading a
	// potentially imcomplete log if the download was interrupted last time. We
	// need to wait so that we don't race to check the _current_download.second
	// status before the downloader marks the file as in-progress.
	std::this_thread::sleep_for(std::chrono::seconds(5));

	while (!_should_exit) {

		if (_loop_disabled) {
			std::this_thread::sleep_for(std::chrono::seconds(1));
			continue;
		}

		// Get list of logs to upload
		auto logs_to_upload = get_logs_to_upload();

		for (const auto& log_path : logs_to_upload) {

			if (_should_exit) {
				return;
			}

			if (_loop_disabled) {
				break;
			}

			if (fs::file_size(log_path) == 0) {
				// TODO: investigate root cause
				// Skip and delete erroneous logs of size zero
				std::cout << "Deleting erroneous zero length log file" << std::endl;
				fs::remove(log_path);
				continue;
			}

			// Upload the log
			if (server_reachable()) {
				if (send_log_to_server(log_path)) {
					mark_log_as_uploaded(log_path);

				} else {
					std::cout << "Sending log to server failed" << std::endl;
				}

			} else {
				std::cout << "Server unreachable" << std::endl;
				break;
			}
		}

		std::this_thread::sleep_for(std::chrono::seconds(5));
	}
}

std::vector<std::string> LogLoader::get_logs_to_upload()
{
	std::vector<std::string> logs;

	for (const auto& it : fs::directory_iterator(_settings.logging_directory)) {
		std::string filename = it.path().filename().string();
		bool should_upload = !log_has_been_uploaded(filename) && log_download_complete(filename);

		if (should_upload) {
			logs.push_back(it.path());
		}
	}

	return logs;
}

bool LogLoader::log_has_been_uploaded(const std::string& filename)
{
	std::ifstream file(_settings.uploaded_logs_file);
	std::string line;

	while (std::getline(file, line)) {
		if (line == filename) {
			return true;
		}
	}

	return false;
}

bool LogLoader::log_download_complete(const std::string& filename)
{
	std::lock_guard<std::mutex> lock(_current_download_mutex);

	if (_current_download.first == filename) {
		return _current_download.second;
	}

	return true;
}

void LogLoader::mark_log_as_uploaded(const std::string& file_path)
{
	std::ofstream file(_settings.uploaded_logs_file, std::ios::app);
	file << std::filesystem::path(file_path).filename().string() << std::endl;
}

std::string LogLoader::filepath_from_entry(const mavsdk::LogFiles::Entry entry)
{
	std::ostringstream ss;
	ss << _settings.logging_directory << "LOG" << std::setfill('0') << std::setw(4) << entry.id << "_" << entry.date << ".ulg";
	return ss.str();
}

bool LogLoader::server_reachable()
{
	httplib::SSLClient cli(_settings.server);
	httplib::Result res = cli.Get("/");;

	// if (_settings.server.find("https") != std::string::npos) {
	// 	httplib::SSLClient cli(_settings.server);
	// 	res = cli.Get("/");

	// } else {
	// 	httplib::Client cli(_settings.server);
	// 	res = cli.Get("/");
	// }

	bool success = res && res->status == 200;

	if (!success) {
		std::cout << "Connection to " << _settings.server << " failed: " << (res ? std::to_string(res->status) : "No response") << std::endl;
	}

	return success;
}

bool LogLoader::send_log_to_server(const std::string& file_path)
{
	std::ifstream file(file_path, std::ios::binary);

	if (!file) {
		std::cout << "Could not open file " << file_path << std::endl;
		return false;
	}

	// Build multi-part form data
	httplib::MultipartFormDataItems items = {
		{"type", _settings.public_logs ? "flightreport" : "personal", "", ""}, // NOTE: backend logic is funky
		{"description", "Uploaded by logloader", "", ""},
		{"feedback", "", "", ""},
		{"email", _settings.email, "", ""},
		{"source", "auto", "", ""},
		{"videoUrl", "", "", ""},
		{"rating", "", "", ""},
		{"windSpeed", "", "", ""},
		{"public", _settings.public_logs ? "true" : "false", "", ""},
	};

	std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
	items.push_back({"filearg", content, file_path, "application/octet-stream"});

	// Post multi-part form
	std::cout << "Uploading:  "
		  << std::setw(24) << std::left << fs::path(file_path).filename().string()
		  << std::setw(8) << std::fixed << std::setprecision(2) << fs::file_size(file_path) / 1e6 << "MB"
		  << std::flush << std::endl;

	httplib::SSLClient cli(_settings.server);
	httplib::Result res =  cli.Post("/upload", items);;

	// if (_settings.server.find("https") != std::string::npos) {
	// 	httplib::SSLClient cli(_settings.server);
	// 	res = cli.Post("/upload", items);

	// } else {
	// 	httplib::Client cli(_settings.server);
	// 	res = cli.Post("/upload", items);
	// }

	if (res && res->status == 302) {
		std::string url = _settings.server + res->get_header_value("Location");
		std::cout << std::endl << "Upload success:" << std::endl << url << std::endl;
		return true;
	}

	else {
		std::cout << "Failed to upload " << file_path << ". Status: " << (res ? std::to_string(res->status) : "No response") << std::endl;
		return false;
	}
}

mavsdk::LogFiles::Entry LogLoader::find_most_recent_log()
{
	mavsdk::LogFiles::Entry entry = {};
	// Regex pattern to match "LOG<index>_<datetime>.ulg" format
	std::regex log_pattern("LOG(\\d+)_(\\d{4}-\\d{2}-\\d{2}T\\d{2}:\\d{2}:\\d{2}Z)\\.ulg");
	int max_index = -1; // Start with -1 to ensure any found index will be greater
	std::string latest_datetime; // To keep track of the latest datetime for the highest index

	for (const auto& dir_iter : fs::directory_iterator(_settings.logging_directory)) {
		std::string filename = dir_iter.path().filename().string();

		std::smatch matches;

		if (std::regex_search(filename, matches, log_pattern) && matches.size() > 2) {
			int index = std::stoi(matches[1].str()); // Index is in the first capture group
			std::string datetime = matches[2].str(); // Datetime is in the second capture group

			// Check if this log has a higher index or same index with a later timestamp
			if (index > max_index || (index == max_index && datetime > latest_datetime)) {
				max_index = index;
				latest_datetime = datetime;
				// construct log Entry
				entry.id = index;
				entry.date = datetime;
				entry.size_bytes = dir_iter.file_size();
			}
		}
	}

	return entry;
}
