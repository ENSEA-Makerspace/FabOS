<?php
require 'vendor/autoload.php';

use PhpMqtt\Client\MqttClient;
use PhpMqtt\Client\ConnectionSettings;

$server   = 'broker.mqttdashboard.com'; // À remplacer
$port     = 1883;
$clientId = 'machine-' . uniqid();
$MachineID = 'machine-42'; // ID unique de la machine

$connectionSettings = new ConnectionSettings();
$connectionSettings->setConnectTimeout(3);
$mqtt = new MqttClient($server, $port, $clientId, $connectionSettings);

// --- 1. Envoyer le statut de la machine (POST) ---
function sendStatus($mqtt, $MachineID, $status) {
    $mqtt->publish(
        "machine/{$MachineID}/status",
        json_encode([
            'MachineID' => $MachineID,
            'status' => $status,
        ]),
        1
    );
}

// --- 2. Notifier le travail (start/stop) (POST) ---
function notifyWork($mqtt, $MachineID, $action) {
    $mqtt->publish(
        "machine/{$MachineID}/work",
        json_encode([
            'IDMachine' => $MachineID,
            'action' => $action,
        ]),
        1
    );
}

// --- 3. Demander une autorisation (GET) ---
function requestAuthorization($mqtt, $MachineID, $IDUserRFID, $callback) {
    $requestId = uniqid();
    $mqtt->publish(
        'machine/authorization/request',
        json_encode([
            'request_id' => $requestId,
            'IDUserRFID' => $IDUserRFID,
            'MachineID' => $MachineID,
        ]),
        1
    );

    // Écoute la réponse
    $mqtt->subscribe("machine/authorization/response/{$requestId}", function ($topic, $message) use ($mqtt, $callback) {
        $response = json_decode($message, true);
        $callback($response['authorized'] ?? false);
        $mqtt->unsubscribe($topic); // Évite les fuites de mémoire
    }, 1);
}

// --- Exemples d'utilisation ---
// 1. Envoyer le statut
sendStatus($mqtt, $MachineID, 'active');

// 2. Notifier le démarrage/arrêt
notifyWork($mqtt, $MachineID, 'start');
// sleep(5);
// notifyWork($mqtt, $MachineID, 'stop');

// 3. Demander une autorisation
requestAuthorization($mqtt, $MachineID, 'user-456', function ($authorized) {
    echo "Autorisation : " . ($authorized ? "ACCORDÉE" : "REFUSÉE") . "\n";
});

// Garde la connexion ouverte pour recevoir les réponses
$mqtt->loop(true);

