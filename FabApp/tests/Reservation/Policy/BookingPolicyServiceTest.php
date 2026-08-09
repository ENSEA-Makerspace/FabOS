<?php

namespace App\Tests\Reservation\Policy;

use App\Entity\Utilisateur;
use App\Repository\ReservationRepository;
use App\Reservation\BookingResult;
use App\Reservation\Policy\BookingPolicyRepository;
use App\Reservation\Policy\BookingPolicyService;
use App\Reservation\ReservableType;
use Doctrine\DBAL\Connection;
use PHPUnit\Framework\TestCase;

final class BookingPolicyServiceTest extends TestCase
{
    public function testPassDoesNotBypassSlotAlignment(): void
    {
        $service = $this->service(['slotIncrementMinutes' => 30]);
        $result = $service->check(new Utilisateur(), ReservableType::Machine, 1, new \DateTimeImmutable('2030-01-01 09:10'), new \DateTimeImmutable('2030-01-01 10:10'), new \DateTimeImmutable('2030-01-01 08:00'), true);

        self::assertInstanceOf(BookingResult::class, $result);
        self::assertSame('SLOT_NOT_ALIGNED', $result->code);
    }

    public function testPassDoesNotBypassResourceBuffer(): void
    {
        $service = $this->service(['bufferMinutes' => 15], overlap: true);
        $result = $service->check(new Utilisateur(), ReservableType::Machine, 1, new \DateTimeImmutable('2030-01-01 09:00'), new \DateTimeImmutable('2030-01-01 10:00'), new \DateTimeImmutable('2030-01-01 08:00'), true);

        self::assertInstanceOf(BookingResult::class, $result);
        self::assertSame('BUFFER_CONFLICT', $result->code);
    }

    public function testDailyCounterReceivesTheReservableType(): void
    {
        $repository = new class extends ReservationRepository {
            public ?ReservableType $countType = null;
            public function __construct() {}
            public function hasOverlap(ReservableType $type, int $id, \DateTimeImmutable $start, \DateTimeImmutable $end, ?int $ignoreId = null): bool { return false; }
            public function countForUserStartingBetween(Utilisateur $user, ReservableType $type, \DateTimeImmutable $from, \DateTimeImmutable $to, ?int $ignoreId = null): int { $this->countType = $type; return 1; }
        };
        $service = $this->service(['maxPerDay' => 1], repository: $repository);
        $result = $service->check(new Utilisateur(), ReservableType::Place, 1, new \DateTimeImmutable('2030-01-01 09:00'), new \DateTimeImmutable('2030-01-01 10:00'), new \DateTimeImmutable('2030-01-01 08:00'));

        self::assertInstanceOf(BookingResult::class, $result);
        self::assertSame('DAILY_LIMIT_REACHED', $result->code);
        self::assertSame(ReservableType::Place, $repository->countType);
    }

    private function service(array $limits, bool $overlap = false, ?ReservationRepository $repository = null): BookingPolicyService
    {
        $connection = $this->createStub(Connection::class);
        $connection->method('fetchAssociative')->willReturn(array_merge([
            'reservableType' => 'machine', 'tier' => 'member',
        ], $limits));
        $repository ??= new class($overlap) extends ReservationRepository {
            public function __construct(private bool $overlap) {}
            public function hasOverlap(ReservableType $type, int $id, \DateTimeImmutable $start, \DateTimeImmutable $end, ?int $ignoreId = null): bool { return $this->overlap; }
        };

        return new BookingPolicyService(new BookingPolicyRepository($connection), $repository);
    }
}
