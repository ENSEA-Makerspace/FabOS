<?php
require 'vendor/autoload.php';

use PhpMqtt\Client\MqttClient;
use PhpMqtt\Client\ConnectionSettings;

// --- Configuration MQTT ---
$server   = 'broker.mqttdashboard.com';
$port     = 1883;
$clientId = 'server-' . uniqid();

$connectionSettings = new ConnectionSettings();
$connectionSettings->setConnectTimeout(3);
$mqtt = new MqttClient($server, $port, $clientId, $connectionSettings);

// --- Configuration Base de Données ---
$dbHost = 'localhost'; // À adapter
$dbName = 'nom_base_de_donnees'; // À adapter
$dbUser = 'utilisateur'; // À adapter
$dbPass = 'mot_de_passe'; // À adapter

try {
    $pdo = new PDO("pgsql:host=$dbHost;dbname=$dbName", $dbUser, $dbPass);
    $pdo->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);
} catch (PDOException $e) {
    die("Erreur de connexion à la base de données: " . $e->getMessage());
}

// --- Fonction pour vérifier les timeouts des machines ---
function checkMachineTimeouts($pdo, $mqtt) {
    $stmt = $pdo->prepare("UPDATE MACHINE
        SET statut = 'idle', updated = CURRENT_TIMESTAMP WHERE statut = 'active'
        AND lastAuthorizationTime < CURRENT_TIMESTAMP - INTERVAL '1 minute' RETURNING machineToken, statut");
    $stmt->execute();
    $updatedMachines = $stmt->fetchAll(PDO::FETCH_ASSOC);

    foreach ($updatedMachines as $machine) {
        echo "[TIMEOUT] Machine {$machine['machineToken']} passée en 'idle' après 1 minute.\n";
        // Optionnel: Envoyer une notification MQTT
        $mqtt->publish(
            "machine/{$machine['machineToken']}/status",
            json_encode(['MachineID' => $machine['machineToken'], 'status' => 'idle']),
            1
        );
    }
}


// --- 1. Répondre aux demandes de statut (GET) ---
$mqtt->subscribe('machine/status/get', function ($topic, $message) use ($mqtt, $pdo) {
    $request = json_decode($message, true);
    $requestId = $request['request_id'];
    $machineToken = $request['MachineID'] ?? null; // On utilise machineToken comme identifiant

    if ($machineToken) {
        // Requête SQL pour récupérer la machine
        $stmt = $pdo->prepare("SELECT id, nom AS name, statut AS status FROM MACHINE WHERE machineToken = ?");
        $stmt->execute([$machineToken]);
        $machine = $stmt->fetch(PDO::FETCH_ASSOC);

        if ($machine) {
            $response = $machine + ['id' => $machine['id']]; // On conserve l'id de la table
        } else {
            $response = ['error' => 'Machine non trouvée'];
        }
    } else {
        $response = ['error' => 'MachineID manquant'];
    }

    $mqtt->publish(
        "machine/status/response/{$requestId}",
        json_encode($response),
        1
    );
}, 1);

// --- 2. Répondre aux demandes d'autorisation (GET) ---
$mqtt->subscribe('machine/authorization/request', function ($topic, $message) use ($mqtt, $pdo) {
    $request = json_decode($message, true);
    $requestId = $request['request_id'];
    $IDUserRFID = $request['IDUserRFID'] ?? null;
    $MachineToken = $request['MachineID'] ?? null;

    if (!$IDUserRFID || !$MachineToken) {
        $authorized = false;
    } else {
        // 1. Récupérer l'ID de la machine via son token
        $stmtMachine = $pdo->prepare("SELECT id FROM MACHINE WHERE machineToken = ?");
        $stmtMachine->execute([$MachineToken]);
        $machine = $stmtMachine->fetch(PDO::FETCH_ASSOC);

        if (!$machine) {
            $authorized = false;
        } else {
            // 2. Récupérer l'ID de l'utilisateur via son RFID
            $stmtUser = $pdo->prepare("SELECT id FROM UTILISATEUR WHERE identifiantRfid = ?");
            $stmtUser->execute([$IDUserRFID]);
            $user = $stmtUser->fetch(PDO::FETCH_ASSOC);

            if (!$user) {
                $authorized = false;
            } else {
                // 3. Vérifier si l'utilisateur est autorisé sur cette machine
                $stmtAuth = $pdo->prepare("SELECT 1 FROM MACHINE_UTILISATEUR WHERE machineId = ? AND userId = ?");
                $stmtAuth->execute([$machine['id'], $user['id']]);
                $authorized = $stmtAuth->fetch() !== false;

                if ($authorized) {
                    $stmtUpdate = $pdo->prepare("
                        UPDATE MACHINE
                        SET statut = 'active',
                            lastAuthorizationTime = CURRENT_TIMESTAMP,
                            updated = CURRENT_TIMESTAMP
                        WHERE machineToken = ?
                    ");
                    $stmtUpdate->execute([$MachineToken]);
                    echo "[AUTORISATION] Machine {$MachineToken} passée en 'active' pour 1 minute.\n";
                }
            }
        }
    }

    $mqtt->publish(
        "machine/authorization/response/{$requestId}",
        json_encode(['authorized' => $authorized]),
        1
    );
}, 1);


// --- 3. Écouter les statuts des machines (POST) ---
$mqtt->subscribe('machine/+/status', function ($topic, $message) use ($pdo) {
    $statusUpdate = json_decode($message, true);
    $machineToken = $statusUpdate['MachineID'] ?? null;
    $newStatus = $statusUpdate['status'] ?? null;

    if ($machineToken && $newStatus) {
        $stmt = $pdo->prepare("UPDATE MACHINE SET statut = ?, updated = CURRENT_TIMESTAMP WHERE machineToken = ?");
        $result = $stmt->execute([$newStatus, $machineToken]);

        if ($stmt->rowCount() > 0) {
            echo "[STATUT] Machine {$machineToken} : statut mis à jour en '{$newStatus}'\n";
        } else {
            echo "[STATUT] Machine non trouvée: {$machineToken}\n";
        }
    } else {
        echo "[STATUT] Données incomplètes dans le message\n";
    }
}, 1);



// --- 4. Écouter les notifications de travail (POST) ---
$mqtt->subscribe('machine/+/work', function ($topic, $message) use ($pdo) {
    $notification = json_decode($message, true);
    $machineToken = $notification['IDMachine'] ?? null;
    $action = $notification['action'] ?? null;
    $userRFID = $notification['IDUserRFID'] ?? null;

    if (!$machineToken || !$action) {
        echo "[TRAVAIL] Données incomplètes: IDMachine ou action manquant\n";
        return;
    }

    // Récupérer l'ID de la machine
    $stmtMachine = $pdo->prepare("SELECT id FROM MACHINE WHERE machineToken = ?");
    $stmtMachine->execute([$machineToken]);
    $machine = $stmtMachine->fetch(PDO::FETCH_ASSOC);

    if (!$machine) {
        echo "[TRAVAIL] Machine non trouvée: {$machineToken}\n";
        return;
    }
    $machineId = $machine['id'];

    if ($action === 'start') {
        if ($machine['statut'] !== 'active') {
            $mqtt->publish(
                "machine/{$machineToken}/command",
                json_encode([
                    'action' => 'stop',
                    'reason' => 'Machine non active (statut: ' . $machine['statut'] . ')',
                    'MachineID' => $machineToken
                ]),
                1
            );
            echo "[TRAVAIL] REFUS: Machine {$machineToken} n'est pas en 'active'. Ordre d'arrêt envoyé.\n";
            return;
        }

        if (!$userRFID) {
            echo "[TRAVAIL] UserRFID manquant pour l'action 'start'\n";
            return;
        }

        // Récupérer l'ID de l'utilisateur
        $stmtUser = $pdo->prepare("SELECT id FROM UTILISATEUR WHERE identifiantRfid = ?");
        $stmtUser->execute([$userRFID]);
        $user = $stmtUser->fetch(PDO::FETCH_ASSOC);

        if (!$user) {
            echo "[TRAVAIL] Utilisateur non trouvé: {$userRFID}\n";
            return;
        }
        $userId = $user['id'];

        // Créer un nouveau log d'utilisation
        $stmtLog = $pdo->prepare("INSERT INTO LOG_UTILISATION (machineId, userId, dateDebut, source, createdAt)
            VALUES (?, ?, CURRENT_TIMESTAMP, 'mqtt', CURRENT_TIMESTAMP)
        ");
        $stmtLog->execute([$machineId, $userId]);

        echo "[TRAVAIL] Machine {$machineToken} a DÉMARRÉ le travail (utilisateur: {$userRFID})\n";

    } elseif ($action === 'stop') {
        // Trouver le dernier log en cours pour cette machine
        $stmtLog = $pdo->prepare("SELECT id, userId, dateDebut
            FROM LOG_UTILISATION
            WHERE machineId = ? AND dateFin IS NULL
            ORDER BY dateDebut DESC
            LIMIT 1
        ");
        $stmtLog->execute([$machineId]);
        $currentLog = $stmtLog->fetch(PDO::FETCH_ASSOC);

        if ($currentLog) {
            // Mettre à jour le log avec dateFin et durée (en minutes)
            $stmtUpdate = $pdo->prepare("UPDATE LOG_UTILISATION
                SET dateFin = CURRENT_TIMESTAMP,
                    duree = EXTRACT(EPOCH FROM (CURRENT_TIMESTAMP - dateDebut)) / 60
                WHERE id = ?
            ");
            $stmtUpdate->execute([$currentLog['id']]);

            // Mettre à jour le temps de présence total de l'utilisateur
            $stmtUpdateUser = $pdo->prepare("UPDATE UTILISATEUR
                SET tempsPresenceTotal = tempsPresenceTotal + (
                    SELECT EXTRACT(EPOCH FROM (CURRENT_TIMESTAMP - dateDebut)) / 60
                    FROM LOG_UTILISATION
                    WHERE id = ?
                )
                WHERE id = ?
            ");
            $stmtUpdateUser->execute([$currentLog['id'], $currentLog['userId']]);

            echo "[TRAVAIL] Machine {$machineToken} a ARRÊTÉ le travail\n";
        } else {
            echo "[TRAVAIL] Aucune session en cours trouvée pour la machine {$machineToken}\n";
        }
    } else {
        echo "[TRAVAIL] Action inconnue: {$action}\n";
    }
}, 1);


// --- Boucle principale avec vérification des timeouts ---
echo "Serveur MQTT en écoute sur tous les topics...\n";
// Utiliser un timestamp pour déclencher checkMachineTimeouts toutes les 1 seconde
$lastTimeoutCheck = time();
while (true) {
    $mqtt->loop(false, 0.1);

    // Vérifier si 1 seconde s'est écoulée depuis la dernière vérification
    if (time() - $lastTimeoutCheck >= 1) {
        checkMachineTimeouts($pdo, $mqtt);
        $lastTimeoutCheck = time();
    }
}